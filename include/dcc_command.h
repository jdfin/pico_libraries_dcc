#pragma once

#include <cstdint>
#include <list>

#include "dcc_bitstream.h"
#include "dcc_pkt2.h"
#include "dcc_throttle.h"
#include "hardware/uart.h"

#undef INCLUDE_ACK_DBG

class DccAdc;

class DccCommand
{
public:

    DccCommand(int sig_gpio, int pwr_gpio, int slp_gpio, DccAdc &adc,
               uart_inst_t *const rc_uart = nullptr, int rc_gpio = -1);
    ~DccCommand();

    void set_mode_off();
    void set_mode_ops();

    void write_cv(int cv_num, uint8_t cv_val);
    void write_bit(int cv_num, int bit_num, int bit_val);
    void read_cv(int cv_num);
    void read_bit(int cv_num, int bit_num);

    // Returns true if service mode operation is done, and result is set
    // true (success) or false (failed). For read operations, use the one
    // with val to get the result.
    bool svc_done(bool &result);
    bool svc_done(bool &result, uint8_t &val);

    enum class Mode {
        OFF,
        OPS,
        SVC,
    };

    Mode mode() const { return _mode; }

    DccAdc &adc() const { return _adc; }

    // called by DccBitstream to get a packet to send
    void get_packet(DccPkt2 &pkt);

    void loop();

    DccThrottle *find_throttle(int address);
    DccThrottle *create_throttle(int address = DccPkt::address_default);
    DccThrottle *delete_throttle(DccThrottle *throttle);
    DccThrottle *delete_throttle(int address);
    void restart_throttles();

    void show();

    void show_dcc(bool show)
    {
        _bitstream.show_dcc(show);
        _show_acks = show;
    }
    bool show_dcc() const { return _bitstream.show_dcc(); }

    void show_railcom(bool show) { _bitstream.show_railcom(show); }
    bool show_railcom() const { return _bitstream.show_railcom(); }

    void show_rc_speed(bool show)
    {
        for (DccThrottle *t : _throttles)
            t->show_rc_speed(show);
    }

    bool show_rc_speed()
    {
        // return true if any throttle has show_rc_speed set
        for (DccThrottle *t : _throttles)
            if (t->show_rc_speed())
                return true;
        return false;
    }

private:

    bool _show_acks;

    DccBitstream _bitstream;

    DccAdc &_adc;

    Mode _mode;

    enum class ModeSvc {
        NONE,
        WRITE_CV,
        WRITE_BIT,
        READ_CV,
        READ_BIT,
    };

    ModeSvc _mode_svc;

    std::list<DccThrottle *> _throttles;
    std::list<DccThrottle *>::iterator _next_throttle;

    void get_packet_ops(DccPkt2 &pkt);

    // used by write_cv(), write_bit(), read_cv(), and read_bit()
    void svc_start();

    // for CV operations
    enum CvOp {
        IN_PROGRESS,
        SUCCESS,
        ERROR,
    };

    CvOp _svc_status, _svc_status_next;

    // When in service mode, we check for ack in the bit loop. _ack_ma is
    // initialized to ack_ma_inv and _ack false. After the initial resets, we
    // set _ack_ma to the long average + ack_inc_ma. In the bit loop, if
    // _ack_ma is not ack_ma_invalid, and the short average is larger than
    // _ack_ma, we set _ack true. The next get_packet_* call sees that,
    // handles it, and sets _ack_ma to ack_ma_invalid.
    uint16_t _ack_ma;
    static constexpr uint16_t ack_ma_inv = UINT16_MAX;
    static constexpr uint16_t ack_inc_ma = 60;
    bool _ack;

    void ack_reset()
    {
        _ack_ma = ack_ma_inv;
        _ack = false;
    }

    // When we start looking for an ack, set the threshold
    void ack_arm(uint16_t ack_ma)
    {
        _ack_ma = ack_ma;
        _ack = false;
    }

    // Look for ack and trigger if we see one
    bool ack_check(uint16_t track_ma)
    {
        if (_ack_ma != ack_ma_inv && track_ma >= _ack_ma) {
            // ack!
            _ack_ma = ack_ma_inv;
            _ack = true;
            return true;
        } else {
            // no ack
            return false;
        }
    }

    // See if we have an ack, and clear it if so
    bool ack()
    {
        if (_ack) {
            ack_reset();
            if (_show_acks) {
                char *b = BufLog::write_line_get();
                if (b != nullptr) {
                    snprintf(b, BufLog::line_len, "<< ACK");
                    BufLog::write_line_put();
                }
            }
            return true;
        } else {
            return false;
        }
    }

    enum class SvcCmdStep {
        NONE,
        RESET1,  // sending initial resets (typ 20)
        COMMAND, // sending write or verify commands (typ 5)
        RESET2,  // sending final resets (typ 5)
    };
    SvcCmdStep _svc_cmd_step;
    int _svc_cmd_cnt;

    DccPktReset _pkt_reset;

    // for service mode write byte or bit
    DccPktSvcWriteCv _pkt_svc_write_cv;
    DccPktSvcWriteBit _pkt_svc_write_bit;
    void get_packet_svc_write(DccPkt2 &pkt);

    // for service mode verify byte or bit
    DccPktSvcVerifyCv _pkt_svc_verify_cv;
    DccPktSvcVerifyBit _pkt_svc_verify_bit;
    int _verify_bit;
    int _verify_bit_val; // 0 or 1
    uint8_t _cv_val;
    void get_packet_svc_read_cv(DccPkt2 &pkt);
    void get_packet_svc_read_bit(DccPkt2 &pkt);

    void assert_svc_idle();

    ///// Debug

    // These are used to assert a GPIO on some event to trigger a scope.
    // All default to -1 (disabled).
    static int dbg_get_packet; // asserted for duration of get_packet()
    static void dbg_init();    // call this after changing any from default

    uint32_t _get_packet_min_us;
    uint32_t _get_packet_max_us;
    uint32_t _get_packet_avg_us;
    static constexpr int get_packet_avg_len = 16;
    void dbg_times_reset()
    {
        _get_packet_min_us = UINT32_MAX;
        _get_packet_max_us = 0;
        _get_packet_avg_us = 0;
    }
};
