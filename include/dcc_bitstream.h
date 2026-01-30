#pragma once

#include "buf_log.h"
#include "dcc_pkt.h"
#include "dcc_pkt2.h"
#include "dcc_spec.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "railcom.h"

class DccCommand;

class DccBitstream
{

public:

    DccBitstream(DccCommand &command, int sig_gpio, int pwr_gpio,
                 uart_inst_t *uart = nullptr, int rc_gpio = -1);
    ~DccBitstream();

    void start_ops();
    void start_svc();
    void stop();

    // log DCC packets sent to BufLog
    bool show_dcc() const
    {
        return _show_dcc;
    }
    void show_dcc(bool en)
    {
        _show_dcc = en;
    }

    // log railcom packets received to BufLog
    bool show_railcom() const
    {
        return _show_railcom;
    }
    void show_railcom(bool en)
    {
        _show_railcom = en;
    }

private:

    bool _show_dcc;
    bool _show_railcom;

    DccCommand &_command;

    RailCom _railcom;

    int _pwr_gpio;

    DccPkt2 _current2;

    int _preamble_bits;

    uint _slice;    // uint to match pico-sdk
    uint _channel;  // uint to match pico-sdk

    static constexpr int byte_num_cutout = -2;
    static constexpr int byte_num_preamble = -1;

    // ...then 0 and up are the data bytes
    int _byte_num;  // -2 for cutout, -1 for preamble, then index in _current2
    int _bit_num;   // counts down bit in cutout, preamble, or data byte

    bool _use_railcom;   // railcom cutout or not

    void start(int preamble_bits, bool cutout = true);

    // PWM programming: we always program a 50% duty cycle, changing the
    // period for zero or one.
    //
    // The PWM frequency is set to 1 MHz at init time. Since the actual period
    // in PWM clocks is 'wrap' + 1, we set 'wrap' to the period in
    // microseconds - 1.
    //
    // The output is high for 'level' clocks, so the half-bit time goes in the
    // level register unchanged.
    //
    // Example: for square wave with period 4 us, wrap=3 and level=2.
    //
    // To support the railcom cutout, a second PWM 'channel' is used. This is
    // likely specific to the RP2xxx PWM device. The first channel creates the
    // bit, and the second channel (which runs perfectly in sync with the
    // first) either has the motor driver on or off for part or all of a bit
    // time. It's on when sending bits, off for much of the cutout, and on for
    // a quarter-bit at the start of the cutout. Cutout timing is always in
    // terms of 'one' bits.

    void prog_bit(int b) // called in interrupt context
    {
        int half_us = (b == 0 ? DccSpec::t0_nom_us : DccSpec::t1_nom_us);
        pwm_set_wrap(_slice, 2 * half_us - 1);
        pwm_set_chan_level(_slice, _channel, half_us);
        // power on
        pwm_set_chan_level(_slice, 1 - _channel, 2 * half_us);
    }

    void prog_bit_cutout_start() // called in interrupt context
    {
        pwm_set_wrap(_slice, 2 * DccSpec::t1_nom_us - 1);
        pwm_set_chan_level(_slice, _channel, DccSpec::t1_nom_us);
        // power on for a quarter-bit (half-bit / 2)
        pwm_set_chan_level(_slice, 1 - _channel, DccSpec::t1_nom_us / 2);
    }

    void prog_bit_cutout() // called in interrupt context
    {
        pwm_set_wrap(_slice, 2 * DccSpec::t1_nom_us - 1);
        pwm_set_chan_level(_slice, _channel, DccSpec::t1_nom_us);
        // power off
        pwm_set_chan_level(_slice, 1 - _channel, 0);
    }

    void next_bit(); // called in interrupt context

    static void pwm_handler(void *arg); // called in interrupt context

    ///// Debug

    // These are used to assert a GPIO on some event to trigger a scope.
    // All default to -1 (disabled).
    static int dbg_next_bit;    // asserted for duration of next_bit()
    static void dbg_init();     // call this after changing any from default

}; // class DccBitstream
