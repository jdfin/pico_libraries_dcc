#pragma once

#include "dcc_pkt.h"
#include "dcc_spec.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"  // uint?

class DccBitstream
{
public:
    DccBitstream(int sig_gpio, int pwr_gpio);
    ~DccBitstream();

    void start_ops(bool railcom=false);
    void start_svc();
    void stop();

    void railcom(bool railcom) { _railcom = railcom; }

    bool railcom() const { return _railcom; }

    inline bool need_packet() { return (_next == &_pkt_idle); }

    void send_packet(const DccPkt &pkt);

    inline void send_reset() { send_packet(_pkt_reset); }

private:
    int _pwr_gpio;

    DccPktIdle _pkt_idle;
    DccPktReset _pkt_reset;
    DccPkt _pkt_a;
    DccPkt _pkt_b;

    // ISR sends packet at _current. When it is done:
    //   _current = _next   // _pkt_a, _pkt_b, _pkt_idle, _pkt_reset
    //   _next = _pkt_idle
    //   start packet at _current with _preamble_bits

    DccPkt *_first;    // first _current to go
    DccPkt *_current;  // never nullptr
    DccPkt *_next;     // _pkt_idle if nothing to send

    int _preamble_bits;

    uint _slice;    // uint to match pico-sdk
    uint _channel;  // uint to match pico-sdk

    bool _railcom;  // do railcom cutout (or not)

    int _byte;  // -1 for preamble, then index in _current
    int _bit;   // counts down bit in preamble or _byte

    void start(int preamble_bits, DccPkt &first);

    inline void prog_bit(int b, int power_qtr=4)
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

};  // class DccBitstream
