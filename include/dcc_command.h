#pragma once

#include <cstdint>
#include <list>

#include "dcc_bitstream.h"

#undef INCLUDE_ACK_DBG

class DccAdc;
class DccThrottle;

class DccCommand
{
public:
    DccCommand(int sig_gpio, int pwr_gpio, int slp_gpio, DccAdc &adc);
    ~DccCommand();

    void mode_off();
    void mode_ops(bool railcom=false);
    void mode_svc_write_cv(int cv_num, uint8_t cv_val);
    void mode_svc_write_bit(int cv_num, int bit_num, int bit_val);
    void mode_svc_read_cv(int cv_num);
    void mode_svc_read_bit(int cv_num, int bit_num);

    enum Mode {
        MODE_OFF,
        MODE_OPS,
        MODE_SVC_WRITE_CV,
        MODE_SVC_WRITE_BIT,
        MODE_SVC_READ_CV,
        MODE_SVC_READ_BIT,
    };

    Mode mode() const { return _mode; }

    // Returns true if service mode operation is done, and result is set
    // true (success) or false (failed). For read operations, use the one
    // with val to get the result.
    bool svc_done(bool &result);
    bool svc_done(bool &result, uint8_t &val);

    void loop();

    DccThrottle *create_throttle();
    void delete_throttle(DccThrottle *throttle);

    void show();

private:
    DccBitstream _bitstream;

    DccAdc &_adc;

    Mode _mode;

    // for MODE_OPS
    std::list<DccThrottle *> _throttles;
    std::list<DccThrottle *>::iterator _next_throttle;
    void loop_ops();

    // for MODE_SVC_*
    enum {
        IN_PROGRESS,
        SUCCESS,
        ERROR,
    } _svc_status, _svc_status_next;
    uint16_t _ack_ma;
    static const uint16_t ack_inc_ma = 60;

    int _reset1_cnt;
    int _reset2_cnt;

    // for MODE_SVC_WRITE_CV
    DccPktSvcWriteCv _pkt_svc_write_cv;
    DccPktSvcWriteBit _pkt_svc_write_bit;
    int _write_cnt;
    void loop_svc_write_cv();
    void loop_svc_write_bit();

    // for MODE_SVC_READ_CV
    DccPktSvcVerifyBit _pkt_svc_verify_bit;
    DccPktSvcVerifyCv _pkt_svc_verify_cv;
    int _verify_bit;
    int _verify_bit_val;  // 0 or 1
    int _verify_cnt;
    uint8_t _cv_val;
    void loop_svc_read_cv();
    void loop_svc_read_bit();

    void assert_svc_idle();
};