#include "dcc_throttle.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "buf_log.h"
#include "dcc_pkt.h"
#include "hardware/timer.h"
#include "railcom_msg.h"
#include "xassert.h"

DccThrottle::DccThrottle(int address) :
    _seq(0),
    _pkt_last(nullptr),
    _read_cv_cnt(0),
    _write_cv_cnt(0),
    _write_bit_cnt(0),
    _ops_cv_done(false),
    _ops_cv_status(false),
    _ops_cv_val(0),
    _speed(0),
    _speed_us(UINT64_MAX),
    _show_speed(false)
{
    set_address(address);
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
#if (DCC_FUNC_MAX >= 21)
    _pkt_func_21.set_address(address);
#endif
#if (DCC_FUNC_MAX >= 29)
    _pkt_func_29.set_address(address);
#endif
#if (DCC_FUNC_MAX >= 37)
    _pkt_func_37.set_address(address);
#endif
#if (DCC_FUNC_MAX >= 45)
    _pkt_func_45.set_address(address);
#endif
#if (DCC_FUNC_MAX >= 53)
    _pkt_func_53.set_address(address);
#endif
#if (DCC_FUNC_MAX >= 61)
    _pkt_func_61.set_address(address);
#endif
    _pkt_read_cv.set_address(address);
    _pkt_write_cv.set_address(address);
    _pkt_write_bit.set_address(address);
    _seq = 0;
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
#if (DCC_FUNC_MAX >= 21)
    } else if (num <= 28) {
        return _pkt_func_21.get_f(num);
#endif
#if (DCC_FUNC_MAX >= 29)
    } else if (num <= 36) {
        return _pkt_func_29.get_f(num);
#endif
#if (DCC_FUNC_MAX >= 37)
    } else if (num <= 44) {
        return _pkt_func_37.get_f(num);
#endif
#if (DCC_FUNC_MAX >= 45)
    } else if (num <= 52) {
        return _pkt_func_45.get_f(num);
#endif
#if (DCC_FUNC_MAX >= 53)
    } else if (num <= 60) {
        return _pkt_func_53.get_f(num);
#endif
#if (DCC_FUNC_MAX >= 61)
    } else if (num <= 68) {
        return _pkt_func_61.get_f(num);
#endif
    } else {
        xassert(false);
        return false;
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
#if (DCC_FUNC_MAX >= 21)
    } else if (num <= 28) {
        _pkt_func_21.set_f(num, on);
        _seq = 9;
#endif
#if (DCC_FUNC_MAX >= 29)
    } else if (num <= 36) {
        _pkt_func_29.set_f(num, on);
        _seq = 11;
#endif
#if (DCC_FUNC_MAX >= 37)
    } else if (num <= 44) {
        _pkt_func_37.set_f(num, on);
        _seq = 13;
#endif
#if (DCC_FUNC_MAX >= 45)
    } else if (num <= 52) {
        _pkt_func_45.set_f(num, on);
        _seq = 15;
#endif
#if (DCC_FUNC_MAX >= 53)
    } else if (num <= 60) {
        _pkt_func_53.set_f(num, on);
        _seq = 17;
#endif
#if (DCC_FUNC_MAX >= 61)
    } else if (num <= 68) {
        _pkt_func_61.set_f(num, on);
        _seq = 19;
#endif
    } else {
        xassert(false);
    }
}

// ops mode cv access

void DccThrottle::read_cv(int cv_num)
{
    _pkt_read_cv.set_cv(cv_num);
    _ops_cv_done = false;
    _ops_cv_status = false;
    // +1 because when it decrements to zero it's an error
    _read_cv_cnt = read_cv_send_cnt + 1;
}

void DccThrottle::write_cv(int cv_num, uint8_t cv_val)
{
    _pkt_write_cv.set_cv(cv_num, cv_val);
    _ops_cv_done = false;
    _ops_cv_status = false;
    _write_cv_cnt = write_cv_send_cnt;
}

void DccThrottle::write_bit(int cv_num, int bit_num, int bit_val)
{
    _pkt_write_bit.set_cv_bit(cv_num, bit_num, bit_val);
    _ops_cv_done = false;
    _ops_cv_status = false;
    _write_bit_cnt = write_bit_send_cnt;
}

bool DccThrottle::ops_done(bool &result, uint8_t &value)
{
    if (!_ops_cv_done)
        return false;

    result = _ops_cv_status;
    value = _ops_cv_val;

    return true;
}

//  0. Speed     1. F0-F4
//  2. Speed     3. F5-F8
//  4. Speed     5. F9-F12
//  6. Speed     7. F13-F20
//  8. Speed     9. F21-F28
// 10. Speed    11. F29-F36
// 12. Speed    13. F37-F44
// 14. Speed    15. F45-F52
// 16. Speed    17. F53-F60
// 18. Speed    19. F61-F68
DccPkt DccThrottle::next_packet()
{
    xassert(0 <= _seq && _seq < seq_max);

    if (_read_cv_cnt > 0) {
        _read_cv_cnt--;
        if (_read_cv_cnt == 0) {
            // No response. Since CV read requires railcom, this is an error.
            _ops_cv_done = true;
            _ops_cv_status = false;
            _ops_cv_val = 0x00; // arbitrary, seems better to set it
            // continue on below to return a different packet
        } else {
            _pkt_last = &_pkt_read_cv;
            return _pkt_read_cv;
        }
    }

    if (_write_cv_cnt > 0) {
        _write_cv_cnt--;
        _pkt_last = &_pkt_write_cv;
        return _pkt_write_cv;
    }

    if (_write_bit_cnt > 0) {
        _write_bit_cnt--;
        _pkt_last = &_pkt_write_bit;
        return _pkt_write_bit;
    }

    int seq = _seq;

    if (++_seq >= seq_max)
        _seq = 0;

    if ((seq & 1) == 0) { // if _seq even
        _pkt_last = &_pkt_speed;
        return _pkt_speed;
    } else if (seq == 1) {
        _pkt_last = &_pkt_func_0;
        return _pkt_func_0;
    } else if (seq == 3) {
        _pkt_last = &_pkt_func_5;
        return _pkt_func_5;
    } else if (seq == 5) {
        _pkt_last = &_pkt_func_9;
        return _pkt_func_9;
    } else if (seq == 7) {
        _pkt_last = &_pkt_func_13;
        return _pkt_func_13;
#if (DCC_FUNC_MAX >= 21)
    } else if (seq == 9) {
        _pkt_last = &_pkt_func_21;
        return _pkt_func_21;
#endif
#if (DCC_FUNC_MAX >= 29)
    } else if (seq == 11) {
        _pkt_last = &_pkt_func_29;
        return _pkt_func_29;
#endif
#if (DCC_FUNC_MAX >= 37)
    } else if (seq == 13) {
        _pkt_last = &_pkt_func_37;
        return _pkt_func_37;
#endif
#if (DCC_FUNC_MAX >= 45)
    } else if (seq == 15) {
        _pkt_last = &_pkt_func_45;
        return _pkt_func_45;
#endif
#if (DCC_FUNC_MAX >= 53)
    } else if (seq == 17) {
        _pkt_last = &_pkt_func_53;
        return _pkt_func_53;
#endif
#if (DCC_FUNC_MAX >= 61)
    } else if (seq == 19) {
        _pkt_last = &_pkt_func_61;
        return _pkt_func_61;
#endif
    } else {
        __builtin_unreachable();
    }
}

// This is called (at interrupt level) if any railcom channel2 messages are
// received in the cutout following a DCC message from this throttle.
void DccThrottle::railcom(const RailComMsg *const msg, int msg_cnt)
{
    constexpr int verbosity = 0;

    // verbosity 9: print all dcc sent and railcom received
    // verbosity 1: print only railcom pom received

    if constexpr (verbosity >= 9) {

        char *b = BufLog::write_line_get();
        if (b == nullptr)
            return;
        char *e = b + BufLog::line_len;

        // show DCC packet sent
        b += snprintf(b, e - b, "{");
        if (_pkt_last != nullptr) {
            _pkt_last->show(b, e - b);
            b += strlen(b);
        }
        b += snprintf(b, e - b, "}");

        // show railcom packet received
        b += snprintf(b, e - b, " {");
        if (msg_cnt == 0) {
            b += snprintf(b, e - b, " no data");
        } else {
            for (int i = 0; i < msg_cnt; i++) {
                b += snprintf(b, e - b, " ");
                b += msg[i].show(b, e - b);
            }
        }
        b += snprintf(b, e - b, "}");

        BufLog::write_line_put();

    } else if constexpr (verbosity >= 1) {

        char *b = nullptr; // get a log line only when needed
        char *e = nullptr;

        for (int i = 0; i < msg_cnt; i++) {
            if (msg[i].id == RailComMsg::MsgId::pom) {
                if (b == nullptr) {
                    b = BufLog::write_line_get();
                    if (b == nullptr)
                        return;
                    e = b + BufLog::line_len;
                } else {
                    // if there are two, put a space between them
                    b += snprintf(b, e - b, " ");
                }
                b += msg[i].show(b, e - b);
            }
        }

        if (b != nullptr)
            BufLog::write_line_put();
    }

    // process messages received

    for (int i = 0; i < msg_cnt; i++) {
        if (msg[i].id == RailComMsg::MsgId::pom) {
            if (_read_cv_cnt > 0) {
                xassert(_write_cv_cnt == 0 && _write_bit_cnt == 0);
                _ops_cv_done = true;
                _ops_cv_status = true;
                _ops_cv_val = msg[i].pom.val;
                _read_cv_cnt = 0;
            } else if (_write_cv_cnt > 0) {
                xassert(_write_bit_cnt == 0);
                _ops_cv_done = true;
                _ops_cv_status = true;
                _ops_cv_val = msg[i].pom.val;
                _write_cv_cnt = 0;
            } else if (_write_bit_cnt > 0) {
                _ops_cv_done = true;
                _ops_cv_status = true;
                _ops_cv_val = msg[i].pom.val;
                _write_bit_cnt = 0;
            }
        } else if (msg[i].id == RailComMsg::MsgId::dyn) {
            if (msg[i].dyn.id == RailComSpec::DynId::dyn_speed_1) {
                if (msg[i].dyn.val != _speed) {
                    // loco's self-reported speed has changed
                    _speed = msg[i].dyn.val;
                    _speed_us = time_us_64();
                    if (_show_speed) {
                        char *b = BufLog::write_line_get();
                        if (b != nullptr) {
                            snprintf(b, BufLog::line_len, "%0.3f speed=%u",
                                     _speed_us / 1000000.0, _speed);
                            BufLog::write_line_put();
                        }
                    }
                }
            }
        }
    }

} // void DccThrottle::railcom(const RailComMsg *msg, int msg_cnt)

void DccThrottle::show()
{
    char buf[80];
    printf("%s\n", _pkt_speed.show(buf, sizeof(buf)));
    printf("%s\n", _pkt_func_0.show(buf, sizeof(buf)));
    printf("%s\n", _pkt_func_5.show(buf, sizeof(buf)));
    printf("%s\n", _pkt_func_9.show(buf, sizeof(buf)));
    printf("%s\n", _pkt_func_13.show(buf, sizeof(buf)));
#if (DCC_FUNC_MAX >= 21)
    printf("%s\n", _pkt_func_21.show(buf, sizeof(buf)));
#endif
#if (DCC_FUNC_MAX >= 29)
    printf("%s\n", _pkt_func_29.show(buf, sizeof(buf)));
#endif
#if (DCC_FUNC_MAX >= 37)
    printf("%s\n", _pkt_func_37.show(buf, sizeof(buf)));
#endif
#if (DCC_FUNC_MAX >= 45)
    printf("%s\n", _pkt_func_45.show(buf, sizeof(buf)));
#endif
#if (DCC_FUNC_MAX >= 53)
    printf("%s\n", _pkt_func_53.show(buf, sizeof(buf)));
#endif
#if (DCC_FUNC_MAX >= 61)
    printf("%s\n", _pkt_func_61.show(buf, sizeof(buf)));
#endif
}
