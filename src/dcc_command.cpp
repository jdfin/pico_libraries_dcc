#include "dcc_command.h"

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <list>

#include "buf_log.h"
#include "dcc_adc.h"
#include "dcc_bitstream.h"
#include "dcc_pkt.h"
#include "dcc_throttle.h"
#include "xassert.h"

static uart_inst_t *rc_uart = uart0;
static int rc_gpio = 17; // uart0 rx
static int dbg_gpio = 21;

DccCommand::DccCommand(int sig_gpio, int pwr_gpio, int slp_gpio, DccAdc &adc) :
    _bitstream(sig_gpio, pwr_gpio, rc_uart, rc_gpio, dbg_gpio),
    _adc(adc),
    _mode(MODE_OFF),
    _next_throttle(_throttles.begin()),
    _svc_status(ERROR),
    _svc_status_next(ERROR),
    _reset1_cnt(0),
    _reset2_cnt(0),
    _pkt_svc_write_cv(),
    _pkt_svc_write_bit(),
    _write_cnt(0),
    _pkt_svc_verify_bit(),
    _pkt_svc_verify_cv(),
    _verify_bit(0),
    _verify_bit_val(0),
    _verify_cnt(0),
    _cv_val(0)
{
    // The way the "two turnouts" board is constructed, it's easier to
    // connect /SLP to a gpio and drive it high rather than add a pullup.
    // A pullup would be fine for all current cases.
    if (slp_gpio >= 0) {
        gpio_init(slp_gpio);
        gpio_put(slp_gpio, 1);
        gpio_set_dir(slp_gpio, GPIO_OUT);
    }
}

DccCommand::~DccCommand()
{
    for (DccThrottle *t : _throttles) {
        _throttles.remove(t);
        delete t;
    }
    _next_throttle = _throttles.begin();
}

void DccCommand::mode_off()
{
    _mode = MODE_OFF;
    _adc.stop();
    _bitstream.stop();
}

void DccCommand::mode_ops()
{
    _mode = MODE_OPS;
    _bitstream.start_ops();
}

void DccCommand::mode_svc_write_cv(int cv_num, uint8_t cv_val)
{
    assert_svc_idle();

    _mode = MODE_SVC_WRITE_CV;
    _svc_status = IN_PROGRESS;
    _svc_status_next = IN_PROGRESS;
    _reset1_cnt = 20;
    _pkt_svc_write_cv.set_cv(cv_num, cv_val);

    _adc.start();
    _bitstream.start_svc();
}

void DccCommand::mode_svc_write_bit(int cv_num, int bit_num, int bit_val)
{
    assert_svc_idle();
    _mode = MODE_SVC_WRITE_BIT;
    _svc_status = IN_PROGRESS;
    _svc_status_next = IN_PROGRESS;
    _reset1_cnt = 20;
    _pkt_svc_write_bit.set_cv_bit(cv_num, bit_num, bit_val);

    _adc.start();
    _bitstream.start_svc();
}

void DccCommand::mode_svc_read_cv(int cv_num)
{
    assert_svc_idle();
    _mode = MODE_SVC_READ_CV;
    _svc_status = IN_PROGRESS;
    _svc_status_next = IN_PROGRESS;
    _reset1_cnt = 20;
    _cv_val = 0;
    _pkt_svc_verify_bit.set_cv_num(cv_num);
    _pkt_svc_verify_cv.set_cv_num(cv_num);

    _adc.start();
    _bitstream.start_svc();
}

void DccCommand::mode_svc_read_bit(int cv_num, int bit_num)
{
    assert_svc_idle();
    _mode = MODE_SVC_READ_BIT;
    _svc_status = IN_PROGRESS;
    _svc_status_next = IN_PROGRESS;
    _reset1_cnt = 20;
    _verify_bit = bit_num;
    _pkt_svc_verify_bit.set_cv_num(cv_num);

    _adc.start();
    _bitstream.start_svc();
}

bool DccCommand::svc_done(bool &result)
{
    if (_svc_status == IN_PROGRESS) {
        return false;
    }

    result = (_svc_status == SUCCESS);
    return true;
}

bool DccCommand::svc_done(bool &result, uint8_t &val)
{
    if (_svc_status == IN_PROGRESS) {
        return false;
    }

    result = (_svc_status == SUCCESS);

    val = _cv_val; // return even if !result

    return true;
}

void DccCommand::loop()
{
    if (_mode == MODE_OFF) {
        ; // nop
    } else if (_mode == MODE_OPS) {
        loop_ops();
    } else if (_mode == MODE_SVC_WRITE_CV) {
        _adc.loop();
        loop_svc_write_cv();
    } else if (_mode == MODE_SVC_WRITE_BIT) {
        _adc.loop();
        loop_svc_write_bit();
    } else if (_mode == MODE_SVC_READ_CV) {
        _adc.loop();
        loop_svc_read_cv();
    } else if (_mode == MODE_SVC_READ_BIT) {
        _adc.loop();
        loop_svc_read_bit();
    }
    const char *p = BufLog::read_line_get();
    if (p != nullptr) {
        printf("%s\n", p);
        BufLog::read_line_put();
    }
}

void DccCommand::loop_ops()
{
    if (_bitstream.need_packet()) {
        if (_next_throttle != _throttles.end()) {
            _bitstream.send_packet((*_next_throttle)->next_packet(),
                                   *_next_throttle);
            _next_throttle++;
            if (_next_throttle == _throttles.end()) {
                _next_throttle = _throttles.begin();
            }
        }
    }
}

void DccCommand::loop_svc_write_cv()
{
    xassert(_mode == MODE_SVC_WRITE_CV);

    if (_reset1_cnt > 0) {
        xassert(_write_cnt == 0);
        xassert(_reset2_cnt == 0);
        if (_bitstream.need_packet()) {
            _bitstream.send_reset();
            _reset1_cnt--;
            if (_reset1_cnt == 0) {
                // The long average adc reading is the baseline for
                // detecting an ack pulse.
                _ack_ma = _adc.long_ma() + ack_inc_ma;
                // done with resets, next send write commands
                _write_cnt = 5;
            }
        }
    } else {
        xassert(_reset1_cnt == 0);
        // Use the short average adc reading to detect an ack.
        if (_adc.short_ma() >= _ack_ma) {
            // Ack! Don't send any more packets, and power off.
            if constexpr (!_adc.logging()) {
                _write_cnt = 0;
                _reset2_cnt = 0;
            }
            // We can't have _svc_status != IN_PROGRESS after returning from
            // this function. Having this 'next' value covers adc logging.
            _svc_status_next = SUCCESS;
        }
        if (_write_cnt > 0) {
            xassert(_reset2_cnt == 0);
            if (_bitstream.need_packet()) {
                _bitstream.send_packet(_pkt_svc_write_cv);
                _write_cnt--;
                if (_write_cnt == 0) {
                    // done with write commands, next send more resets
                    _reset2_cnt = 5;
                }
            }
        } else if (_reset2_cnt > 0) {
            xassert(_write_cnt == 0);
            if (_bitstream.need_packet()) {
                _bitstream.send_reset();
                _reset2_cnt--;
            }
        } else {
            xassert(_write_cnt == 0);
            xassert(_reset2_cnt == 0);
            if (_svc_status_next == IN_PROGRESS) {
                _svc_status = ERROR; // failed, timeout
            } else {
                _svc_status = SUCCESS;
            }
            mode_off();
        }
    }
} // void DccCommand::loop_svc_write_cv()

void DccCommand::loop_svc_write_bit()
{
    xassert(_mode == MODE_SVC_WRITE_BIT);

    if (_reset1_cnt > 0) {
        xassert(_write_cnt == 0);
        xassert(_reset2_cnt == 0);
        if (_bitstream.need_packet()) {
            _bitstream.send_reset();
            _reset1_cnt--;
            if (_reset1_cnt == 0) {
                // The long average adc reading is the baseline for
                // detecting an ack pulse.
                _ack_ma = _adc.long_ma() + ack_inc_ma;
                // done with resets, next send write commands
                _write_cnt = 5;
            }
        }
    } else {
        xassert(_reset1_cnt == 0);
        // Use the short average adc reading to detect an ack.
        uint16_t short_ma = _adc.short_ma();
        if (short_ma >= _ack_ma) {
            // Ack! Don't send any more packets, and power off.
            if constexpr (!_adc.logging()) {
                _write_cnt = 0;
                _reset2_cnt = 0;
            }
            // We can't have _svc_status != IN_PROGRESS after returning from
            // this function. Having this 'next' value covers adc logging.
            _svc_status_next = SUCCESS;
        }
        if (_write_cnt > 0) {
            xassert(_reset2_cnt == 0);
            if (_bitstream.need_packet()) {
                _bitstream.send_packet(_pkt_svc_write_bit);
                _write_cnt--;
                if (_write_cnt == 0) {
                    // done with write commands, next send more resets
                    _reset2_cnt = 5;
                }
            }
        } else if (_reset2_cnt > 0) {
            xassert(_write_cnt == 0);
            if (_bitstream.need_packet()) {
                _bitstream.send_reset();
                _reset2_cnt--;
            }
        } else {
            xassert(_write_cnt == 0);
            xassert(_reset2_cnt == 0);
            if (_svc_status_next == IN_PROGRESS) {
                _svc_status = ERROR; // failed, timeout
            } else {
                _svc_status = SUCCESS;
            }
            mode_off();
        }
    }
} // void DccCommand::loop_svc_write_bit()

// Before the first call (when starting the read), mode_svc_read_cv() sets:
//   _reset1_cnt to the number of initial resets (20)
//   _cv_val = 0, so this loop can OR-in one bits as they are discovered
//   _svc_status = IN_PROGRESS, to indicate the read is in progress
//
// As the loop is repeatedly called:
//   1. it will send out the initial resets
//   2. it will, for each bit 7...0:
//      a. send out five bit-verifies (that the bit is one)
//      b. send out five resets
//      c. and if an ack is received during any of those 10 packets, a one bit
//         is ORed into _cv_val
//   3. after the verify-bit for bit 0, it sends out five byte-verifies for
//      the cv with the built-up _cv_val, then five more resets
//      a. if an ack is received during any of those 10 packets, we are done,
//         _svc_status is set to SUCCESS, and calling svc_done() will
//         return "done/success" and the cv_val
//      b. if no ack has been received when the last reset goes out, we are
//         done, _svc_status is set to ERROR, and calling svc_done() will
//         return "done/error"

void DccCommand::loop_svc_read_cv()
{
    xassert(_mode == MODE_SVC_READ_CV);

    if (_reset1_cnt > 0) {
        // first 20 resets are going out

        xassert(_verify_cnt == 0);
        xassert(_reset2_cnt == 0);

        if (!_bitstream.need_packet()) {
            return;
        }

        _bitstream.send_reset();
        _reset1_cnt--;
        if (_reset1_cnt == 0) {
            // Done with resets. Use the long average adc reading as the
            // baseline for detecting an ack pulse.
            _ack_ma = _adc.long_ma() + ack_inc_ma;
            _verify_bit = 7;
            _verify_bit_val = 1;
            _verify_cnt = 5;
            _pkt_svc_verify_bit.set_bit(_verify_bit, _verify_bit_val);
        }
        return;
    }

    xassert(_reset1_cnt == 0);

    if (_adc.short_ma() >= _ack_ma) {
        // Ack!
        if (_verify_bit < 8) {
            // This is an ack for a bit-verify
            _cv_val |= (1 << _verify_bit);
        } else {
            // This is the ack for the byte-verify at the end
            _svc_status_next = SUCCESS;
            // If logging adc (for analysis), we keep going to see the
            // full ack. If not logging, we're done.
            if constexpr (!_adc.logging()) {
                // this makes it so no more packets are sent below
                _verify_cnt = 0;
                _reset2_cnt = 0;
            }
        }
    }

    if (_verify_cnt > 0) {
        xassert(_reset2_cnt == 0);
        if (_bitstream.need_packet()) {
            if (_verify_bit == 8) {
                _bitstream.send_packet(_pkt_svc_verify_cv);
            } else {
                _bitstream.send_packet(_pkt_svc_verify_bit);
            }
            _verify_cnt--;
            if (_verify_cnt == 0) {
                _reset2_cnt = 5;
            }
        }
    } else if (_reset2_cnt > 0) {
        xassert(_verify_cnt == 0);
        if (_bitstream.need_packet()) {
            _bitstream.send_reset();
            _reset2_cnt--;
            if (_reset2_cnt == 0) {
                // Get a new long average adc reading and a new ack threshold
                // each time just before sending out the verify packets. The
                // current might not always hold steady through the whole
                // sequence.
                _ack_ma = _adc.long_ma() + ack_inc_ma;
                // printf("long_ma = %u, ack_ma = %u\n", long_ma, _ack_ma);
            }
        }
    } else {
        // done with 5 verifies and 5 resets for _verify_bit
        xassert(_verify_cnt == 0);
        xassert(_reset2_cnt == 0);
        if (_verify_bit == 8) {
            // done with the byte verify at the end
            if (_svc_status_next == IN_PROGRESS) {
                _svc_status = ERROR; // failed, timeout
            } else {
                _svc_status = SUCCESS;
            }
            mode_off();
        } else if (_verify_bit > 0) {
            // done with a single-bit verify
            _verify_bit--;
            xassert(_verify_bit >= 0 && _verify_bit <= 7);
            xassert(_verify_bit_val == 1);
            _pkt_svc_verify_bit.set_bit(_verify_bit, _verify_bit_val);
            _verify_cnt = 5;
        } else {
            xassert(_verify_bit == 0);
            // start byte verify
            _verify_bit = 8; // signifies verify byte
            _pkt_svc_verify_cv.set_cv_val(_cv_val);
            _verify_cnt = 5;
        }
    }

} // void DccCommand::loop_svc_read_cv

void DccCommand::loop_svc_read_bit()
{
    xassert(_mode == MODE_SVC_READ_BIT);

    if (_reset1_cnt > 0) {
        // first 20 resets are going out

        xassert(_verify_cnt == 0);
        xassert(_reset2_cnt == 0);

        if (!_bitstream.need_packet()) {
            return;
        }

        _bitstream.send_reset();
        _reset1_cnt--;
        if (_reset1_cnt == 0) {
            // Done with resets (second-to-last one has just started)
            _ack_ma = _adc.long_ma() + ack_inc_ma;
            // printf("long_ma = %u, ack_ma = %u\n", long_ma, _ack_ma);
            xassert(_verify_bit >= 0 && _verify_bit <= 7);
            _verify_bit_val = 0; // first 0, then 1 if no ack for 0
            _verify_cnt = 5;
            _pkt_svc_verify_bit.set_bit(_verify_bit, _verify_bit_val);
        }
        return;
    }

    xassert(_reset1_cnt == 0);

    if (_adc.short_ma() >= _ack_ma) {
        // Ack! Could be checking for 0 or for 1.
        _cv_val = _verify_bit_val;
        // Either way we're done
        _svc_status_next = SUCCESS;
        // If logging adc (for analysis), we keep going to see the
        // full ack. If not logging, we're done.
        if constexpr (!_adc.logging()) {
            // this makes it so no more packets are sent below
            _verify_cnt = 0;
            _reset2_cnt = 0;
        }
    }

    if (_verify_cnt > 0) {
        xassert(_reset2_cnt == 0);
        if (_bitstream.need_packet()) {
            _bitstream.send_packet(_pkt_svc_verify_bit);
            _verify_cnt--;
            if (_verify_cnt == 0) {
                _reset2_cnt = 5;
            }
        }
    } else if (_reset2_cnt > 0) {
        xassert(_verify_cnt == 0);
        if (_bitstream.need_packet()) {
            _bitstream.send_reset();
            _reset2_cnt--;
        }
    } else {
        // done with 5 verifies and 5 resets for _verify_bit
        xassert(_verify_cnt == 0);
        xassert(_reset2_cnt == 0);
        if (_svc_status_next == IN_PROGRESS && _verify_bit_val == 0) {
            // tried 0, got no ack, try 1
            _verify_bit_val = 1;
            _verify_cnt = 5;
            _pkt_svc_verify_bit.set_bit(_verify_bit, _verify_bit_val);
        } else {
            // tried 0, then 1; hopefully got an ack for one of them
            if (_svc_status_next == IN_PROGRESS) {
                _svc_status = ERROR; // didn't get an ack for either
            } else {
                _svc_status = SUCCESS;
            }
            mode_off();
        }
    }

} // void DccCommand::loop_svc_read_bit

DccThrottle *DccCommand::find_throttle(int address)
{
    if (address < DccPkt::address_min || address > DccPkt::address_max) {
        return nullptr;
    }

    for (DccThrottle *t : _throttles) {
        if (t->get_address() == address) {
            return t;
        }
    }

    // address not found
    return nullptr;
}

DccThrottle *DccCommand::create_throttle(int address)
{
    if (address < DccPkt::address_min || address > DccPkt::address_max) {
        return nullptr;
    }

    DccThrottle *throttle = find_throttle(address);
    if (throttle == nullptr) {
        throttle = new DccThrottle(address);
        _throttles.push_back(throttle);
        restart_throttles();
    }

    return throttle;
}

DccThrottle *DccCommand::delete_throttle(DccThrottle *throttle)
{
    _throttles.remove(throttle);
    delete throttle;
    restart_throttles();
    return *_throttles.begin();
}

DccThrottle *DccCommand::delete_throttle(int address)
{
    for (DccThrottle *throttle : _throttles) {
        if (throttle->get_address() == address) {
            delete_throttle(throttle);
            return *_throttles.begin();
        }
    }
    // not found
    return *_throttles.begin();
}

void DccCommand::restart_throttles()
{
    _throttles.sort([](const DccThrottle *a, const DccThrottle *b) {
        return a->get_address() < b->get_address();
    });

    for (DccThrottle *t : _throttles) {
        t->restart();
    }

    _next_throttle = _throttles.begin();
}

void DccCommand::show()
{
    if (_throttles.empty()) {
        printf("no throttles\n");
    } else {
        for (DccThrottle *throttle : _throttles) {
            printf("throttle:\n");
            throttle->show();
        }
    }
}

void DccCommand::assert_svc_idle()
{
    xassert(_mode == MODE_OFF);
    xassert(_svc_status != IN_PROGRESS);
    xassert(_svc_status_next != IN_PROGRESS);
    xassert(_reset1_cnt == 0);
    xassert(_write_cnt == 0);
    xassert(_verify_cnt == 0);
    xassert(_reset2_cnt == 0);
}
