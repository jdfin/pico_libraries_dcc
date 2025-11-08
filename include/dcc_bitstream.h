#pragma once

#include "buf_log.h"
#include "dcc_pkt.h"
#include "dcc_pkt2.h"
#include "dcc_spec.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "railcom.h"

class DccBitstream
{

public:

    DccBitstream(int sig_gpio, int pwr_gpio, uart_inst_t *uart = nullptr,
                 int rc_gpio = -1, int dbg_gpio = -1);
    ~DccBitstream();

    void start_ops();
    void start_svc();
    void stop();

    bool need_packet()
    {
        return (_next2 == &_pkt2_idle);
    }

    void send_packet(const DccPkt &pkt, DccThrottle *throttle = nullptr);

    void send_reset()
    {
        send_packet(_pkt_reset);
    }

private:

    RailCom _railcom;

    int _pwr_gpio;

    int _dbg_gpio;

    DccPktIdle _pkt_idle;
    DccPktReset _pkt_reset;
    DccPkt2 _pkt2_idle;
    DccPkt2 _pkt2_reset;
    DccPkt2 _pkt2_a;
    DccPkt2 _pkt2_b;

    // ISR sends packet at _current2. When packet is done:
    //   _current2 = _next2 (one of _pkt2_a, _pkt2_b, _pkt2_idle, _pkt2_reset)
    //   _next2 = &_pkt2_idle
    //   start packet at _current2 with _preamble_bits

    DccPkt2 *_current2;
    DccPkt2 *_next2; // never nullptr; &_pkt2_idle if nothing to send

    int _preamble_bits;

    uint _slice;   // uint to match pico-sdk
    uint _channel; // uint to match pico-sdk

    int _byte; // -1 for preamble, then index in _current2
    int _bit;  // counts down bit in preamble or _byte

    void start(int preamble_bits, DccPkt &first);

    void prog_bit(int b, int power_qtr = 4)
    {
        // Period is wrap+1 usec (pwm_hz = 1 MHz), and output is high for
        // count=[0...level-1], low for count=[level...wrap].
        // Set wrap to bit_us-1 and level to bit_us/2.
        // E.g. for square wave with period 4 us, wrap=3 and level=2.

        int half_us = (b == 0 ? DccSpec::t0_nom_us : DccSpec::t1_nom_us);
        pwm_set_wrap(_slice, 2 * half_us - 1);
        pwm_set_chan_level(_slice, _channel, half_us);

        // power_qtr is the quarter-bits the power is on for:
        // 4 for most bits (preamble, message)
        // 1 for the first cutout bit (on for half a bit then off)
        // 0 for the other cutout bits (off)
        pwm_set_chan_level(_slice, 1 - _channel, (power_qtr * half_us) / 2);
    }

    void next_bit();

    static void pwm_handler(void *arg);

}; // class DccBitstream
