#pragma once

#include <cstdint>

#include "dcc_pkt.h"

class RailComMsg;

class DccThrottle
{

public:

    DccThrottle(int address = DccPkt::address_default);
    ~DccThrottle();

    int get_address() const;
    void set_address(int address);

    int get_speed() const;
    void set_speed(int speed);

    bool get_function(int func) const;
    void set_function(int func, bool on);

    void read_cv(int cv_num);
    void read_bit(int cv_num, int bit_num);

    void write_cv(int cv_num, uint8_t cv_val);
    void write_bit(int cv_num, int bit_num, int bit_val);

    DccPkt next_packet();

    void railcom(const RailComMsg *msg, int msg_cnt);

    // reset packet sequence to start (typically for debug purposes)
    void restart()
    {
        _seq = 0;
    }

    void show();

private:

    DccPktSpeed128 _pkt_speed; // sent if seq even (0, 2, ... 16, 18)
    DccPktFunc0 _pkt_func_0;   // seq == 1
    DccPktFunc5 _pkt_func_5;   // seq == 3
    DccPktFunc9 _pkt_func_9;   // seq == 5
    DccPktFunc13 _pkt_func_13; // seq == 7
#if INCLUDE_DCC_FUNC_21
    DccPktFunc21 _pkt_func_21; // seq == 9
#if INCLUDE_DCC_FUNC_29
    DccPktFunc29 _pkt_func_29; // seq == 11
#if INCLUDE_DCC_FUNC_37
    DccPktFunc37 _pkt_func_37; // seq == 13
#if INCLUDE_DCC_FUNC_45
    DccPktFunc45 _pkt_func_45; // seq == 15
#if INCLUDE_DCC_FUNC_53
    DccPktFunc53 _pkt_func_53; // seq == 17
#if INCLUDE_DCC_FUNC_61
    DccPktFunc61 _pkt_func_61; // seq == 19
#endif
#endif
#endif
#endif
#endif
#endif

    // where in packet sequence we are

#if INCLUDE_DCC_FUNC_61
    static const int seq_max = 20;
#elif INCLUDE_DCC_FUNC_53
    static const int seq_max = 18;
#elif INCLUDE_DCC_FUNC_45
    static const int seq_max = 16;
#elif INCLUDE_DCC_FUNC_37
    static const int seq_max = 14;
#elif INCLUDE_DCC_FUNC_29
    static const int seq_max = 12;
#elif INCLUDE_DCC_FUNC_21
    static const int seq_max = 10;
#else
    static const int seq_max = 8;
#endif

    int _seq; // _seq = 0 ... seq_max-1

    // last packet returned by next_packet
    DccPkt *_pkt_last;

    DccPktOpsReadCv _pkt_read_cv;
    static const int read_cv_send_cnt = 5; // how many times to send it
    int _read_cv_cnt;                      // times left to send it (5, 4, ... 1, 0)

#if 0
    DccPktOpsReadBit _pkt_read_bit;
    static const int read_bit_send_cnt = 5; // how many times to send it
    int _read_bit_cnt; // times left to send it (5, 4, ... 1, 0)
#endif

    DccPktOpsWriteCv _pkt_write_cv;
    static const int write_cv_send_cnt = 5; // how many times to send it
    int _write_cv_cnt;                      // times left to send it (5, 4, ... 1, 0)

    DccPktOpsWriteBit _pkt_write_bit;
    static const int write_bit_send_cnt = 5; // how many times to send it
    int _write_bit_cnt;                      // times left to send it (5, 4, ... 1, 0)

}; // class DccThrottle
