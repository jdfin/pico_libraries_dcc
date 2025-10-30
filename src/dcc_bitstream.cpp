#include "dcc_bitstream.h"

#include <climits>
#include <cstdint>
#include <cstdio>

#include "dbg_gpio.h"  // misc/include/
#include "dcc_pkt.h"
#include "dcc_railcom.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pwm_irq_mux.h"  // misc/include/
#include "xassert.h"

#define LOG_DCC     1
#define LOG_RAILCOM 1

// PWM usage:
//
// Example: sending 0, 1, 1
//
//   |<--------0-------->|<----1---->|<----1---->|
//
//   +---------+         +-----+     +-----+     +--
//   |         |         |     |     |     |     |
// --+         +---------+     +-----+     +-----+
//   ^                   ^           ^           ^
//   A                   B           C           D
//
// At edge A, the PWM's CC and TOP registers are already programmed for the
// zero bit (done at the start of the bit ending at A). The interrupt handler
// called because of the wrap at edge A programs CC and TOP for the one bit
// that will start at edge B. Because of the double-buffering in CC and TOP,
// those values are not used until edge B.
//
// At edge B, the PWM's CC and TOP registers start using the values set at
// edge A. The handler called because of the wrap at edge B programs CC and
// TOP for the one bit starting at edge C.
//
// The railcom cutout is done synchronously with the bitstream by using the
// other channel of the DCC signal's PWM slice. A slice is programmed with a
// certain period, and each channel in the slice can have its own duty cycle.
// So the slice's period is set for the bit period (116 or 200 usec), the
// signal channel has a 50% duty cycle, and the enable channel is either 0%
// (track off, or railcom cutout) or 100% (track on and sending a packet).
// This means the DCC signal and enable GPIOs must be on pins that can be used
// by different channels of the same PWM slice (section 4.5.2 of the RP2040
// datasheet). It does not matter which is channel A and which is channel B.

DccBitstream::DccBitstream(int sig_gpio, int pwr_gpio, uart_inst_t* uart, int rc_gpio) :
    _railcom(uart, rc_gpio),
    _pwr_gpio(pwr_gpio),
    _pkt_idle(),
    _pkt_reset(),
    _pkt_a(),
    _pkt_b(),
    _current(nullptr),
    _next(&_pkt_idle),
    _preamble_bits(DccPkt::ops_preamble_bits),
    _slice(pwm_gpio_to_slice_num(sig_gpio)),
    _channel(pwm_gpio_to_channel(sig_gpio)),
    _byte(INT_MAX),  // set in start_*()
    _bit(INT_MAX),   // set in start_*()
    _log_put(0),
    _log_get(0)
{
    // DbgGpio::init({0});

    // Do not do PWM setup here since this might be a static object, and
    // other stuff is not fully initialized. In particular, clock_get_hz()
    // might return the wrong value. Getting _slice and _channel in the
    // initialization above is okay since that does not require anything
    // else to be initialized.

    gpio_set_function(sig_gpio, GPIO_FUNC_PWM);

    // power gpio must be same PWM slice as signal gpio
    xassert(pwm_gpio_to_slice_num(pwr_gpio) == _slice);
    xassert(pwm_gpio_to_channel(pwr_gpio) == (1 - _channel));

    gpio_set_function(pwr_gpio, GPIO_FUNC_PWM);

}

DccBitstream::~DccBitstream()
{
    stop();  // track power off, pwm output low
}

void DccBitstream::start_ops()
{
    start(DccPkt::ops_preamble_bits, _pkt_idle);
}

void DccBitstream::start_svc()
{
    start(DccPkt::svc_preamble_bits, _pkt_reset);
}

void DccBitstream::start(int preamble_bits, DccPkt& first)
{
    uint32_t sys_hz = clock_get_hz(clk_sys);
    const uint32_t pwm_hz = 1000000;  // 1 MHz; 1 usec/count
    uint32_t pwm_div = sys_hz / pwm_hz;

    // If this is a start after a previous stop, the pwm is not disabled,
    // it's just running a 0% duty cycle waveform.
    pwm_set_enabled(_slice, false);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, pwm_div);
    pwm_init(_slice, &config, false);

    // RP2040 has one pwm with interrupt number PWM_IRQ_WRAP.
    // RP2350 has two pwms with interrupt numbers PWM_IRQ_WRAP_[01],
    // and PWM_IRQ_WRAP is PWM_IRQ_WRAP_0.
    pwm_irq_mux_connect(_slice, pwm_handler, this);  // misc/src/pwm_irq_mux.c
    pwm_clear_irq(_slice);
    pwm_set_irq_enabled(_slice, true);

    _preamble_bits = preamble_bits;

    //          start_ops()      or     start_svc()
    xassert(&first == &_pkt_idle || &first == &_pkt_reset);

    _current = nullptr;
    _next = &first;

    // first packet starts with preamble (no cutout)
    _byte = -1;
    _bit = _preamble_bits - 1;  // will send a preamble bit for
    // _bit = _preamble_bits-1...0

    next_bit();

    pwm_set_enabled(_slice, true);

    // The first bit of the preamble has started going out.
    // Program for second bit when first bit finishes.
    // This assumes the RP2040's double-buffering of TOP and LEVEL.
    next_bit();
}

void DccBitstream::stop()
{
    pwm_set_irq_enabled(_slice, false);
    // stop with output low (0% duty)
    pwm_set_chan_level(_slice, _channel, 0);
    pwm_set_chan_level(_slice, 1 - _channel, 0);  // enable low
    // Let the pwm keep running so it gets to the end of the current bit and
    // switches to the 0% duty cycle. If the bitstream starts again, it'll be
    // disabled while it is initialized.
}

void DccBitstream::send_packet(const DccPkt& pkt)
{
    pwm_set_irq_enabled(_slice, false);

    // make sure the irq is really disabled before changing _next and _pkt_a/b
    __dmb();

    if (_current == &_pkt_a) {
        _pkt_b = pkt;  // copy packet (only 12 bytes)
        _next = &_pkt_b;
    } else {
        _pkt_a = pkt;  // copy
        _next = &_pkt_a;
    }

    // make sure _next and _pkt_a/b are in memory before enabling the irq
    __dmb();

    pwm_set_irq_enabled(_slice, true);
}

// called from start(), then the PWM IRQ handler
void DccBitstream::next_bit()
{
    // byte=-2 is the cutout,
    // byte=-1 is the preamble,
    // then byte=0,1,...msg_len-1 for the message bytes
    if (_byte == -2) {
        // doing railcom cutout
        // _railcom should always be true here
        if (_bit == 4) {
            // first bit, power is on for a quarter bit time
            prog_bit(1, 1);
            _bit--;
            // reset uart in case it got glitched
            _railcom.reset();
        } else if (_bit > 0) {
            // continue cutout
            prog_bit(1, 0);
            _bit--;
        } else {
            // end of cutout, start preamble
            xassert(_bit == 0);
            prog_bit(1, 4);
            _byte = -1;
            _bit = _preamble_bits - 1;  // just did the first preamble bit
        }
    } else if (_byte == -1) {
        // sending preamble
        if (_bit == 0) {
            // end of preamble, send packet start bit
            prog_bit(0);
            _byte = 0;
            _bit = 8 - 1;
        } else if (_bit == (_preamble_bits - 1)) {
            // The cutout just ended and we've started the first preamble bit.
            // The first preamble bit was programmed on the previous interrupt.
            _railcom.read();
            // and continue preamble
            prog_bit(1);
            _bit--;
#if LOG_RAILCOM
            // On the first call from start(), _current is nullptr.
            // Thereafter, _current is always set from _next, so is never nullptr.
            if (_current != nullptr) {
                _railcom.channelize();
                _railcom.show(_log[_log_put], log_line_len);
                if (++_log_put >= log_line_cnt)
                    _log_put = 0;
            }
#endif
        } else {
            // continue preamble
            prog_bit(1);
            _bit--;
        }
    } else {
        // sending message bytes
        xassert(_byte >= 0);
        if (_byte == 0 && _bit == (8 - 1)) {
            // time for the first bit of the first byte
            _current = _next;
            _next = &_pkt_idle;
        }
        int msg_len = _current->msg_len();
        // _bit = 7...0, then -1 means stop bit
        if (_bit == -1) {
            // send stop bit
            if ((_byte + 1) == msg_len) {
                // end of message, send 1
                prog_bit(1);
#if LOG_DCC
                // _current is the message we just sent the stop bit for
                _current->show(_log[_log_put], log_line_len);
                if (++_log_put >= log_line_cnt)
                    _log_put = 0;
#endif
#if 1 // railcom
                // cutout first, then preamble
                _byte = -2;
                _bit = 4;
#else // no railcom
                _byte = -1;  // preamble
                // stop bit counts as first bit of next preamble
                // will do _preamble_bits-2...0 more
                _bit = _preamble_bits - 2;
#endif
            } else {
                // more bytes in message, send 0
                prog_bit(0);
                _byte++;
                _bit = 8 - 1;
            }
        } else {
            int b = (_current->data(_byte) >> _bit) & 1;
            prog_bit(b);
            _bit--;
        }
    }
}

// interrupt handler
void DccBitstream::pwm_handler(void* arg)
{
    // DbgGpio g(0);

    DccBitstream* me = (DccBitstream*)arg;

    me->next_bit();
}