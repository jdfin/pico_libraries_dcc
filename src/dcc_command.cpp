#include "dcc_command.h"

#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <list>

#include "buf_log.h"
#include "dcc_adc.h"
#include "dcc_bitstream.h"
#include "dcc_pkt.h"
#include "dcc_throttle.h"
#include "hardware/uart.h"


// gpio to assert while in the next_bit function
int DccCommand::dbg_get_packet __attribute((weak)) = -1;


DccCommand::DccCommand(int sig_gpio, int pwr_gpio, int slp_gpio, DccAdc &adc,
                       uart_inst_t *const rc_uart, int rc_gpio) :
    _bitstream(*this, sig_gpio, pwr_gpio, rc_uart, rc_gpio),
    _adc(adc),
    _mode(Mode::OFF),
    _mode_svc(ModeSvc::NONE),
    _next_throttle(_throttles.begin()),
    _svc_status(ERROR),
    _svc_status_next(ERROR),
    _svc_cmd_step(SvcCmdStep::NONE),
    _svc_cmd_cnt(0),
    _pkt_reset(),
    _pkt_svc_write_cv(),
    _pkt_svc_write_bit(),
    _pkt_svc_verify_cv(),
    _pkt_svc_verify_bit(),
    _verify_bit(0),
    _verify_bit_val(0),
    _cv_val(0)
{
    if (slp_gpio >= 0) {
        gpio_init(slp_gpio);
        gpio_put(slp_gpio, 1);
        gpio_set_dir(slp_gpio, GPIO_OUT);
    }
    ack_reset();
    dbg_init();
    dbg_times_reset();
}


DccCommand::~DccCommand()
{
    for (DccThrottle *t : _throttles) {
        _throttles.remove(t);
        delete t;
    }
    _next_throttle = _throttles.begin();
}


void DccCommand::dbg_init()
{
    DbgGpio::init(dbg_get_packet);
}


void DccCommand::set_mode_off()
{
    _mode = Mode::OFF;
    _mode_svc = ModeSvc::NONE;
    _adc.stop();
    _bitstream.stop();
}


void DccCommand::set_mode_ops()
{
    _mode = Mode::OPS;
    _mode_svc = ModeSvc::NONE;
    _bitstream.start_ops();
}


void DccCommand::write_cv(int cv_num, uint8_t cv_val)
{
    _pkt_svc_write_cv.set_cv(cv_num, cv_val);
    _mode_svc = ModeSvc::WRITE_CV;
    svc_start();
}


void DccCommand::write_bit(int cv_num, int bit_num, int bit_val)
{
    _pkt_svc_write_bit.set_cv_bit(cv_num, bit_num, bit_val);
    _mode_svc = ModeSvc::WRITE_BIT;
    svc_start();
}


void DccCommand::read_cv(int cv_num)
{
    _cv_val = 0;
    _pkt_svc_verify_bit.set_cv_num(cv_num);
    _pkt_svc_verify_cv.set_cv_num(cv_num);
    _mode_svc = ModeSvc::READ_CV;
    svc_start();
}


void DccCommand::read_bit(int cv_num, int bit_num)
{
    _verify_bit = bit_num;
    _pkt_svc_verify_bit.set_cv_num(cv_num);
    _mode_svc = ModeSvc::READ_BIT;
    svc_start();
}


void DccCommand::svc_start()
{
    assert_svc_idle();
    _mode = Mode::SVC;
    _svc_status = IN_PROGRESS;
    _svc_status_next = IN_PROGRESS;
    assert(_svc_cmd_step == SvcCmdStep::NONE);
    assert(_svc_cmd_cnt == 0);
    _svc_cmd_step = SvcCmdStep::RESET1;
    _svc_cmd_cnt = DccSpec::svc_reset1_cnt;
    _adc.start();
    _bitstream.start_svc();
}


bool DccCommand::svc_done(bool &result)
{
    if (_svc_status == IN_PROGRESS)
        return false;

    result = (_svc_status == SUCCESS);
    return true;
}


bool DccCommand::svc_done(bool &result, uint8_t &val)
{
    if (_svc_status == IN_PROGRESS)
        return false;

    result = (_svc_status == SUCCESS);

    val = _cv_val; // return even if !result

    return true;
}


void DccCommand::loop() // called in interrupt context
{
    if (_mode != Mode::SVC)
        return;

    if (!_adc.loop())
        return; // no new adc samples

    ack_check(_adc.short_avg_ma());
}


void DccCommand::get_packet(DccPkt2 &pkt2) // called in interrupt context
{
    DbgGpio d(dbg_get_packet);
    uint32_t start_us = time_us_32();

    if (_mode == Mode::OPS) {
        get_packet_ops(pkt2);
    } else if (_mode == Mode::SVC) {
        if (_mode_svc == ModeSvc::WRITE_CV || _mode_svc == ModeSvc::WRITE_BIT) {
            get_packet_svc_write(pkt2);
        } else if (_mode_svc == ModeSvc::READ_CV) {
            get_packet_svc_read_cv(pkt2);
        } else {
            assert(_mode_svc == ModeSvc::READ_BIT);
            get_packet_svc_read_bit(pkt2);
        }
    }

    // measure how long this function took for debug/analysis
    uint32_t time_us = time_us_32() - start_us;
    if (time_us < _get_packet_min_us)
        _get_packet_min_us = time_us;
    if (time_us > _get_packet_max_us)
        _get_packet_max_us = time_us;
    if (_get_packet_avg_us == 0)
        _get_packet_avg_us = time_us * get_packet_avg_len;
    else
        _get_packet_avg_us =
            (_get_packet_avg_us * (get_packet_avg_len - 1) + time_us) /
            get_packet_avg_len;
}


void DccCommand::get_packet_ops(DccPkt2 &pkt2) // called in interrupt context
{
    if (_next_throttle == _throttles.end()) {
        // XXX no throttles - return an idle packet
        // XXX how did we get into ops mode?
        assert(false);
    } else {
        pkt2.set((*_next_throttle)->next_packet(), *_next_throttle);
        _next_throttle++;
        if (_next_throttle == _throttles.end())
            _next_throttle = _throttles.begin();
    }
}


// Service mode, write CV (byte or bit)
// 1. Send out DccSpec::svc_reset1_cnt (20) resets
// 2. Send out DccSpec::svc_command_cnt (5) commands (write byte/bit)
// 3. Send out DccSpec::svc_reset2_cnt (5) resets
// If an ack is detected in step 2 or 3, immediately quit and power off track.
void DccCommand::get_packet_svc_write(DccPkt2 &pkt2) // called in interrupt context
{
    assert(_svc_cmd_step != SvcCmdStep::NONE);

    if (_svc_cmd_step == SvcCmdStep::RESET1) {
        assert(_svc_cmd_cnt > 0);
        pkt2.set(_pkt_reset);
        _svc_cmd_cnt--;
        if (_svc_cmd_cnt == 0) {
            // Done with resets (second-to-last one has just started).
            // The long average adc reading is the baseline for
            // detecting an ack pulse.
            ack_arm(_adc.long_avg_ma() + ack_inc_ma);
            // Next send write command.
            _svc_cmd_step = SvcCmdStep::COMMAND;
            _svc_cmd_cnt = DccSpec::svc_command_cnt;
        }
        return;
    }

    // Sending the writes or final resets - check for ack.
    if (ack()) {
        // Don't send any more packets, and power off.
        if (!_adc.logging()) {
            _svc_cmd_step = SvcCmdStep::RESET2;
            _svc_cmd_cnt = 0;
        }
        // We can't have _svc_status != IN_PROGRESS after returning from
        // this function. Having this 'next' value covers adc logging.
        _svc_status_next = SUCCESS;
    }

    if (_svc_cmd_step == SvcCmdStep::COMMAND) {
        assert(_svc_cmd_cnt > 0);
        if (_mode_svc == ModeSvc::WRITE_CV) {
            pkt2.set(_pkt_svc_write_cv);
        } else {
            assert(_mode_svc == ModeSvc::WRITE_BIT);
            pkt2.set(_pkt_svc_write_bit);
        }
        _svc_cmd_cnt--;
        if (_svc_cmd_cnt == 0) {
            // done with write commands, next send more resets
            _svc_cmd_step = SvcCmdStep::RESET2;
            _svc_cmd_cnt = DccSpec::svc_reset2_cnt;
        }
        return;
    }

    assert(_svc_cmd_step == SvcCmdStep::RESET2);

    if (_svc_cmd_cnt > 0) {
        pkt2.set(_pkt_reset);
        _svc_cmd_cnt--;
        return;
    }

    assert(_svc_cmd_cnt == 0);

    if (_svc_status_next == IN_PROGRESS)
        _svc_status = ERROR; // no ack, failed
    else
        _svc_status = SUCCESS;

    set_mode_off();

    _svc_cmd_step = SvcCmdStep::NONE;

} // void DccCommand::get_packet_svc_write(DccPkt2 &pkt2)


// Before the first call (when starting the read), read_cv() sets:
//   _svc_cmd_step to RESET1
//   _svc_cmd_cnt to DccSpec::svc_reset1_cnt (20)
//   _cv_val = 0x00, so this loop can OR-in one bits as they are discovered
//   _svc_status = IN_PROGRESS, to indicate the read is in progress
//
// As the loop is repeatedly called:
//   1. it will send out the 20 initial resets
//   2. it will, for each bit 7..0:
//      a. send out 5 bit-verifies (that the bit is one)
//      b. send out 5 resets
//      c. and if an ack is received during any of those 10 packets, a one bit
//         is ORed into _cv_val
//   3. after the last verify-bit (for bit 0), it sends out five byte-verifies
//      for the cv with the built-up _cv_val, then five more resets
//      a. if an ack is received during any of those 10 packets, we are done,
//         _svc_status is set to SUCCESS, and calling svc_done() will
//         return "done/success" and the cv_val
//      b. if no ack has been received when the last reset goes out, we are
//         done, _svc_status is set to ERROR, and calling svc_done() will
//         return "done/error"
void DccCommand::get_packet_svc_read_cv(DccPkt2 &pkt2) // called in interrupt context
{
    assert(_svc_cmd_step != SvcCmdStep::NONE);

    if (_svc_cmd_step == SvcCmdStep::RESET1) {
        assert(_svc_cmd_cnt > 0);
        pkt2.set(_pkt_reset);
        _svc_cmd_cnt--;
        if (_svc_cmd_cnt == 0) {
            // Done with resets (second-to-last one has just started).
            // The long average adc reading is the baseline for
            // detecting an ack pulse.
            ack_arm(_adc.long_avg_ma() + ack_inc_ma);
            // Now start bit-verifies for each bit in the CV.
            _verify_bit = 7;
            _verify_bit_val = 1;
            _pkt_svc_verify_bit.set_bit(_verify_bit, _verify_bit_val);
            _svc_cmd_step = SvcCmdStep::COMMAND;
            _svc_cmd_cnt = DccSpec::svc_command_cnt;
        }
        return;
    }

    if (ack()) {
        if (_verify_bit < 8) {
            // This is an ack for a bit-verify
            _cv_val |= (1 << _verify_bit);
            // It is probably okay to not send any more bit-verifies for the
            // current bit and start the resets. It might even be possible to
            // skip the resets and start the next bit verify. But for now we
            // just keep going. It could save a few packets per 1-bit someday.
        } else {
            // This is the ack for the byte-verify at the end
            // Don't send any more packets, and power off.
            if (!_adc.logging()) {
                _svc_cmd_step = SvcCmdStep::RESET2;
                _svc_cmd_cnt = 0;
            }
            _svc_status_next = SUCCESS;
        }
    }

    if (_svc_cmd_step == SvcCmdStep::COMMAND) {
        assert(_svc_cmd_cnt > 0);
        if (_verify_bit == 8)
            pkt2.set(_pkt_svc_verify_cv);
        else
            pkt2.set(_pkt_svc_verify_bit);
        _svc_cmd_cnt--;
        if (_svc_cmd_cnt == 0) {
            _svc_cmd_step = SvcCmdStep::RESET2;
            _svc_cmd_cnt = DccSpec::svc_reset2_cnt;
        }
        return;
    }

    assert(_svc_cmd_step == SvcCmdStep::RESET2);

    if (_svc_cmd_cnt > 0) {
        pkt2.set(_pkt_reset);
        _svc_cmd_cnt--;
        if (_svc_cmd_cnt == 0) {
            // Get a new long average adc reading and a new ack threshold
            // each time just before sending out the verify packets. The
            // current might not always hold steady through the whole
            // sequence.
            ack_arm(_adc.long_avg_ma() + ack_inc_ma);
        }
        return;
    }

    // Done with DccSpec::svc_command_cnt verifies and DccSpec::svc_reset2_cnt
    // resets for one of the 8 bit verifies, or the final byte verify.

    assert(_svc_cmd_cnt == 0);

    if (_verify_bit >= 1 && _verify_bit <= 7) {
        // Done with one of the first 7 single-bit verifies;
        // start the next bit verify.
        _verify_bit--;
        assert(_verify_bit >= 0 && _verify_bit <= 7);
        assert(_verify_bit_val == 1);
        _pkt_svc_verify_bit.set_bit(_verify_bit, _verify_bit_val);
        pkt2.set(_pkt_svc_verify_bit);
        _svc_cmd_step = SvcCmdStep::COMMAND;
        _svc_cmd_cnt = DccSpec::svc_command_cnt - 1;
        return;
    }

    if (_verify_bit == 0) {
        // Done with the last single-bit verify;
        // start the final byte verify.
        _verify_bit = 8; // magic number signifies verify byte
        _pkt_svc_verify_cv.set_cv_val(_cv_val);
        pkt2.set(_pkt_svc_verify_cv);
        _svc_cmd_step = SvcCmdStep::COMMAND;
        _svc_cmd_cnt = DccSpec::svc_command_cnt - 1;
        return;
    }

    assert(_verify_bit == 8);

    // Done with the byte verify at the end.
    if (_svc_status_next == IN_PROGRESS)
        _svc_status = ERROR; // no ack, failed
    else
        _svc_status = SUCCESS;

    set_mode_off();

    _svc_cmd_step = SvcCmdStep::NONE;

} // void DccCommand::get_packet_svc_read_cv(DccPkt2 &pkt2)


void DccCommand::get_packet_svc_read_bit(DccPkt2 &pkt2) // called in interrupt context
{
    assert(_svc_cmd_step != SvcCmdStep::NONE);

    if (_svc_cmd_step == SvcCmdStep::RESET1) {
        assert(_svc_cmd_cnt > 0);
        pkt2.set(_pkt_reset);
        _svc_cmd_cnt--;
        if (_svc_cmd_cnt == 0) {
            // Done with resets (second-to-last one has just started).
            // The long average adc reading is the baseline for
            // detecting an ack pulse.
            ack_arm(_adc.long_avg_ma() + ack_inc_ma);
            // Next send bit-verify command.
            _svc_cmd_step = SvcCmdStep::COMMAND;
            _svc_cmd_cnt = DccSpec::svc_command_cnt;
            // Configure the bit-verify packet for the bit of interest.
            assert(_verify_bit >= 0 && _verify_bit <= 7);
            _verify_bit_val = 0; // first 0, then 1 if no ack for 0
            _pkt_svc_verify_bit.set_bit(_verify_bit, _verify_bit_val);
        }
        return;
    }

    // Sending the writes or final resets - check for ack.
    if (ack()) {
        // Don't send any more packets, and power off.
        if (!_adc.logging()) {
            _svc_cmd_step = SvcCmdStep::RESET2;
            _svc_cmd_cnt = 0;
        }
        // Could be checking for 0 or for 1. Either way we're done.
        _cv_val = _verify_bit_val;
        _svc_status_next = SUCCESS;
    }

    if (_svc_cmd_step == SvcCmdStep::COMMAND) {
        assert(_svc_cmd_cnt > 0);
        pkt2.set(_pkt_svc_verify_bit);
        _svc_cmd_cnt--;
        if (_svc_cmd_cnt == 0) {
            // done with verify commands, next send more resets
            _svc_cmd_step = SvcCmdStep::RESET2;
            _svc_cmd_cnt = DccSpec::svc_reset2_cnt;
        }
        return;
    }

    assert(_svc_cmd_step == SvcCmdStep::RESET2);

    if (_svc_cmd_cnt > 0) {
        pkt2.set(_pkt_reset);
        _svc_cmd_cnt--;
        return;
    }

    assert(_svc_cmd_cnt == 0);

    // Done with (typ) 5 bit-verifies and (typ) 5 resets. If that was the
    // first bit we tried (0) and we didn't get an ack, try verifying a 1.

    if (_svc_status_next == IN_PROGRESS && _verify_bit_val == 0) {
        // tried 0, got no ack, try 1
        _verify_bit_val = 1;
        _pkt_svc_verify_bit.set_bit(_verify_bit, _verify_bit_val);
        pkt2.set(_pkt_svc_verify_bit);
        _svc_cmd_step = SvcCmdStep::COMMAND;
        _svc_cmd_cnt = DccSpec::svc_command_cnt;
        return;
    }

    // tried 0, then 1; hopefully got an ack for one of them
    if (_svc_status_next == IN_PROGRESS)
        _svc_status = ERROR; // didn't get an ack for either
    else
        _svc_status = SUCCESS;

    set_mode_off();

    _svc_cmd_step = SvcCmdStep::NONE;

} // void DccCommand::get_packet_svc_read_bit(DccPkt2 &pkt2)


DccThrottle *DccCommand::find_throttle(int address)
{
    if (address < DccPkt::address_min || address > DccPkt::address_max) {
        return nullptr;
    }

    for (DccThrottle *t : _throttles)
        if (t->get_address() == address)
            return t;

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
    assert(_mode == Mode::OFF);
    assert(_svc_status != IN_PROGRESS);
    assert(_svc_status_next != IN_PROGRESS);
    assert(_svc_cmd_step == SvcCmdStep::NONE);
    assert(_svc_cmd_cnt == 0);
}
