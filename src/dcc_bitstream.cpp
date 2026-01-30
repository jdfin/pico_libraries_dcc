#include "dcc_bitstream.h"

#include <cassert>
#include <climits>
#include <cstdint>
#include <cstdio>

#include "buf_log.h"
#include "dbg_gpio.h" // misc/include
#include "dcc_command.h"
#include "dcc_pkt.h"
#include "dcc_throttle.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pwm_irq_mux.h" // misc/include
#include "railcom.h"

#define LOG_DCC 1
#define LOG_RAILCOM 1

// gpio to assert while in the next_bit function
int DccBitstream::dbg_next_bit __attribute((weak)) = -1;

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


DccBitstream::DccBitstream(DccCommand &command, int sig_gpio, int pwr_gpio,
                           uart_inst_t *uart, int rc_gpio) :
    _show_dcc(false),
    _show_railcom(false),
    _command(command),
    _railcom(uart, rc_gpio),
    _pwr_gpio(pwr_gpio),
    _preamble_bits(DccPkt::ops_preamble_bits),
    _slice(pwm_gpio_to_slice_num(sig_gpio)),
    _channel(pwm_gpio_to_channel(sig_gpio)),
    _byte_num(INT_MAX), // set in start_*()
    _bit_num(INT_MAX),  // set in start_*()
    _use_railcom(false)
{
    // Do not do PWM setup here since this might be a static object, and
    // other stuff is not fully initialized. In particular, clock_get_hz()
    // might return the wrong value. Getting _slice and _channel in the
    // initialization above is okay since that does not require anything
    // else to be initialized.

    gpio_set_function(sig_gpio, GPIO_FUNC_PWM);

    // power gpio must be same PWM slice as signal gpio
    assert(pwm_gpio_to_slice_num(pwr_gpio) == _slice);
    assert(pwm_gpio_to_channel(pwr_gpio) == (1 - _channel));

    gpio_set_function(pwr_gpio, GPIO_FUNC_PWM);

    dbg_init();
}


DccBitstream::~DccBitstream()
{
    stop(); // track power off, pwm output low
}


void DccBitstream::dbg_init()
{
    DbgGpio::init(dbg_next_bit);
}


void DccBitstream::start_ops()
{
    start(DccPkt::ops_preamble_bits, true);
}


void DccBitstream::start_svc()
{
    start(DccPkt::svc_preamble_bits, false);
}


void DccBitstream::start(int preamble_bits, bool cutout)
{
    uint32_t sys_hz = clock_get_hz(clk_sys);
    const uint32_t pwm_hz = 1000000; // 1 MHz; 1 usec/count
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
    pwm_irq_mux_connect(_slice, pwm_handler, this); // misc/src/pwm_irq_mux.c
    pwm_clear_irq(_slice);
    pwm_set_irq_enabled(_slice, true);

    _preamble_bits = preamble_bits;
    _use_railcom = cutout;

    // first packet starts with preamble (no cutout, whether enabled or not)
    _byte_num = byte_num_preamble;
    _bit_num = _preamble_bits;

    next_bit();

    // _bit_num = _preamble_bits - 1

    pwm_set_enabled(_slice, true);

    // The first bit of the preamble has just started going out.
    // Program for second bit when first bit finishes.
    // This assumes the RP2040's double-buffering of TOP and LEVEL.
    next_bit();

    // _bit_num = _preamble_bits - 2

} // void DccBitstream::start(int preamble_bits, DccPkt &first)


void DccBitstream::stop()
{
    pwm_set_irq_enabled(_slice, false);
    // stop with output low (0% duty)
    pwm_set_chan_level(_slice, _channel, 0);
    pwm_set_chan_level(_slice, 1 - _channel, 0); // enable low

    // Let the pwm keep running so it gets to the end of the current bit and
    // switches to the 0% duty cycle. If the bitstream starts again, it'll be
    // disabled while it is initialized.

} // void DccBitstream::stop()


// Called from start(), then the PWM IRQ handler in response to the end of
// each bit. When this is called, a new bit has already started. Programming
// in here affects the next bit, the one that will start at the next
// interrupt.
//
// byte=-2 is the railcom cutout
// byte=-1 is the packet preamble
// then byte=0,1...msg_len-1 for the message bytes
//
void DccBitstream::next_bit() // called in interrupt context
{
    DbgGpio g(dbg_next_bit);

    if (_byte_num == byte_num_cutout) {
        // doing railcom cutout
        if (_bit_num == 4) {
            // first bit, power is on for a quarter bit time
            prog_bit_cutout_start();
            _bit_num--;
            // reset uart in case it got glitched
            _railcom.reset();
        } else if (_bit_num > 0) {
            // continue cutout
            prog_bit_cutout();
            _bit_num--;
        } else {
            assert(_bit_num == 0);
            // end of cutout, start preamble
            prog_bit(1); // first bit in preamble
            _byte_num = byte_num_preamble;
            _bit_num = _preamble_bits - 1;
            // Note: All the prog_bit(1) calls (after the first) when sending
            // the preamble are not needed since the PWM will send ones and
            // interrupt until we change it. But prog_bit(1) only takes about
            // 1 usec, so just leave it there.
        }
    } else if (_byte_num == byte_num_preamble) {
        // sending preamble
        if (_bit_num > 0) {
            prog_bit(1);
            if (_bit_num == (_preamble_bits - 1)) {
                if (_show_dcc) {
                    // show DCC packet just sent
                    char *b = BufLog::write_line_get();
                    if (b != nullptr) {
                        char *e = b + BufLog::line_len;
                        b += snprintf(b, e - b, ">> ");
                        _current2.show(b, e - b);
                        BufLog::write_line_put();
                    }
                }
                if (_use_railcom) {
                    // The cutout just ended and we've started the first preamble bit.
                    _railcom.read();
                    _railcom.parse();
                    if (_show_railcom) {
                        // show railcom packet just received
                        char *b = BufLog::write_line_get();
                        if (b != nullptr) {
                            char *e = b + BufLog::line_len;
                            b += snprintf(b, e - b, "<< ");
                            _railcom.show(b, e - b);
                            BufLog::write_line_put();
                        }
                    }
                    // _current2 changes at the end of the preamble
                    DccThrottle *throttle = _current2.get_throttle();
                    if (throttle != nullptr) {
                        const RailComMsg *msg;
                        int msg_cnt = _railcom.get_ch2_msgs(msg);
                        throttle->railcom(msg, msg_cnt);
                    }
                }
            }
            _bit_num--;
        } else {
            // end of preamble, send packet start bit
            assert(_bit_num == 0);
            prog_bit(0);
            _byte_num = 0; // first data byte
            _bit_num = 7;  // data goes msb first
            // get the next packet to send from DccCommand
            _command.get_packet(_current2);
        }
    } else {
        assert(0 <= _byte_num);
        // _bit_num can be more than 7 when sending preamble, but not here
        assert(-1 <= _bit_num && _bit_num <= 7);
        // sending message bytes; _byte_num counts 0...msg_len-1
        int msg_len = _current2.len();
        assert(_byte_num < msg_len);
        // _bit_num = 7...0, then -1 means stop bit
        if (_bit_num == -1) {
            // sent a byte, send stop bit
            if ((_byte_num + 1) == msg_len) {
                // end of message, send message-stop bit
                prog_bit(1);
                if (_use_railcom) {
                    // cutout first, then message preamble
                    _byte_num = byte_num_cutout;
                    _bit_num = 4;
                } else {
                    _byte_num = byte_num_preamble; // message preamble
                    // stop bit counts as first bit of next preamble
                    // will do _preamble_bits-2...0 more
                    _bit_num = _preamble_bits - 1;
                }
            } else {
                // more bytes in message, send byte-stop bit
                prog_bit(0);
                _byte_num++;
                _bit_num = 7;
            }
        } else {
            assert(0 <= _bit_num && _bit_num <= 7);
            int b = (_current2.data(_byte_num) >> _bit_num) & 1;
            prog_bit(b);
            _bit_num--;
        }
    }
    _command.loop();

    // Demonstrate taking more than a bit time in this processing, showing
    // that the next interrupt happens immediately on return and things work
    // okay.
    //busy_wait_us_32(150); // more than DccSpec::t1_nom_us * 2 = 116 usec

} // void DccBitstream::next_bit()


// interrupt handler
void DccBitstream::pwm_handler(void *arg) // called in interrupt context
{
    DccBitstream *me = (DccBitstream *)arg;

    me->next_bit();
}
