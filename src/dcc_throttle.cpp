#include "dcc_throttle.h"
#include "dcc_pkt.h"
#include "xassert.h"
#include <cstdint>
#include <cstdio>

DccThrottle::DccThrottle()
    : _pkt_speed()
    , _pkt_func_0()
    , _pkt_func_5()
    , _pkt_func_9()
    , _pkt_func_13()
    , _pkt_func_21()
    , _seq(0)
    , _pkt_write_cv()
    , _write_cv_cnt(0)
    , _pkt_write_bit()
    , _write_bit_cnt(0)
{
}

DccThrottle::~DccThrottle()
{
}

int DccThrottle::get_address() const
{
    return _pkt_speed.get_address();
}

void DccThrottle::set_address(int address)
{
    _pkt_speed.set_address(address);
    _pkt_func_0.set_address(address);
    _pkt_func_5.set_address(address);
    _pkt_func_9.set_address(address);
    _pkt_func_13.set_address(address);
    _pkt_func_21.set_address(address);
    _seq = 0;
    _pkt_write_cv.set_address(address);
    _pkt_write_bit.set_address(address);
}

int DccThrottle::get_speed() const
{
    return _pkt_speed.get_speed();
}

void DccThrottle::set_speed(int speed)
{
    _pkt_speed.set_speed(speed);
    _seq &= ~1; // back up one if a function packet is next
}

bool DccThrottle::get_function(int num) const
{
    xassert(DccPkt::function_min <= num && num <= DccPkt::function_max);

    if (num <= 4) {
        return _pkt_func_0.get_f(num);
    } else if (num <= 8) {
        return _pkt_func_5.get_f(num);
    } else if (num <= 12) {
        return _pkt_func_9.get_f(num);
    } else if (num <= 20) {
        return _pkt_func_13.get_f(num);
    } else { // num <= 28
        return _pkt_func_21.get_f(num);
    }
}

void DccThrottle::set_function(int num, bool on)
{
    xassert(DccPkt::function_min <= num && num <= DccPkt::function_max);

    if (num <= 4) {
        _pkt_func_0.set_f(num, on);
        _seq = 1;
    } else if (num <= 8) {
        _pkt_func_5.set_f(num, on);
        _seq = 3;
    } else if (num <= 12) {
        _pkt_func_9.set_f(num, on);
        _seq = 5;
    } else if (num <= 20) {
        _pkt_func_13.set_f(num, on);
        _seq = 7;
    } else { // num <= 28
        _pkt_func_21.set_f(num, on);
        _seq = 9;
    }
}

void DccThrottle::write_cv(int cv_num, uint8_t cv_val)
{
    _pkt_write_cv.set_cv(cv_num, cv_val);
    _write_cv_cnt = write_cv_send_cnt;
}

void DccThrottle::write_bit(int cv_num, int bit_num, int bit_val)
{
    _pkt_write_bit.set_cv_bit(cv_num, bit_num, bit_val);
    _write_bit_cnt = write_bit_send_cnt;
}

// 0. Speed     1. F0-F4
// 2. Speed     3. F5-F8
// 4. Speed     5. F9-F12
// 6. Speed     7. F13-F20
// 8. Speed     9. F21-F28
DccPkt DccThrottle::next_packet()
{
    xassert(0 <= _seq && _seq < seq_max);

    if (_write_cv_cnt > 0) {
        _write_cv_cnt--;
        return _pkt_write_cv;
    }

    if (_write_bit_cnt > 0) {
        _write_bit_cnt--;
        return _pkt_write_bit;
    }

    int seq = _seq;

    if (++_seq >= seq_max)
        _seq = 0;

    if ((seq & 1) == 0) // if _seq even
        return _pkt_speed;
    else if (seq == 1)
        return _pkt_func_0;
    else if (seq == 3)
        return _pkt_func_5;
    else if (seq == 5)
        return _pkt_func_9;
    else if (seq == 7)
        return _pkt_func_13;
    else
        return _pkt_func_21;
}

void DccThrottle::show()
{
    char buf[80];
    printf("%s\n", _pkt_speed.show(buf, sizeof(buf)));
    printf("%s\n", _pkt_func_0.show(buf, sizeof(buf)));
    printf("%s\n", _pkt_func_5.show(buf, sizeof(buf)));
    printf("%s\n", _pkt_func_9.show(buf, sizeof(buf)));
    printf("%s\n", _pkt_func_13.show(buf, sizeof(buf)));
    printf("%s\n", _pkt_func_21.show(buf, sizeof(buf)));
}