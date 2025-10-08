#pragma once

#include <climits>
#include <cstdint>
#include <cstring>

class DccPkt
{
public:
    DccPkt(const uint8_t *msg = nullptr, int msg_len = 0);

    ~DccPkt()
    {
        memset(_msg, 0, msg_max);
        _msg_len = 0;
    }

    enum PktType {
        Invalid,
        Reset,
        Ccc0,  // 2.3.1 Decoder and Consist Control
        Speed128,
        Speed28,
        Func0,
        Func5,
        Func9,
        Func13,
        Func21,
        OpsWriteCv,
        OpsWriteBit,
        SvcWriteCv,
        SvcWriteBit,
        SvcVerifyCv,
        SvcVerifyBit,
        Accessory,
        Reserved,
        Advanced,
        Idle,
        Unimplemented,
    };

    virtual PktType get_type() const { return Invalid; }

    void msg_len(int new_len);

    int msg_len() const { return _msg_len; }

    uint8_t data(int idx) const;

    int get_address() const;
    virtual int set_address(int adrs);
    int get_address_size() const;

    void set_xor();

    bool check_xor() const;
    static bool check_xor(const uint8_t *msg, int msg_len);

    // loco address constraints
    static const int address_min = 1;          // 0 is broadcast
    static const int address_short_max = 127;  // 0x7f
    static const int address_max = 10239;      // 0x27ff
    static const int address_inv = INT_MAX;

    static const int speed_min = -127;
    static const int speed_max = 127;
    static const int speed_inv = INT_MAX;

    static const int function_min = 0;
    static const int function_max = 28;

    static const int cv_num_min = 1;
    static const int cv_num_max = 1024;
    static const int cv_num_inv = INT_MAX;

    // CV values can be specified as int8_t (-127..128) or uint8_t (0..255)
    static const int cv_val_min = -127;
    static const int cv_val_max = 255;
    static const int cv_val_inv = INT_MAX;

    static bool is_svc_direct(const uint8_t *msg, int msg_len);

    char *dump(char *buf, int buf_len) const;
    char *show(char *buf, int buf_len) const;

    // DCC Spec 9.2, section A ("preamble")
    static const int ops_preamble_bits = 14;

    // DCC Spec 9.2.3, section E ("long preamble")
    static const int svc_preamble_bits = 20;

    static PktType decode_type(const uint8_t *msg, int msg_len);

    // not sure what the "correct" way to do these is
    // return true and fill in the parameters if the packet is of the correct
    // type
    bool decode_speed_128(int &speed) const;
    bool decode_func_0(int *f) const;   // f[5] is f0..f4
    bool decode_func_5(int *f) const;   // f[4] is f5..f8
    bool decode_func_9(int *f) const;   // f[4] is f9..f12
    bool decode_func_13(int *f) const;  // f[8] is f13..f20
    bool decode_func_21(int *f) const;  // f[8] is f21..f28

protected:
    static const int msg_max = 8;

    uint8_t _msg[msg_max];

    int _msg_len;

    bool check_len_min(char *&b, char *e, int min_len) const;
    bool check_len_is(char *&b, char *e, int len) const;
    void show_cv_access(char *&b, char *e, uint8_t instr, int idx) const;

    static PktType decode_payload(const uint8_t *pay, int pay_len);

};  // DccPkt

// 2.1 - Address Partitions - Idle Packet
class DccPktIdle : public DccPkt
{
public:
    DccPktIdle();
    virtual PktType get_type() const override { return Idle; }
};

// 2.3.1.1 - Decoder Control
class DccPktReset : public DccPkt
{
public:
    DccPktReset();
    virtual PktType get_type() const override { return Reset; }
};

// 2.3.2.1 - 128 Speed Step Control
class DccPktSpeed128 : public DccPkt
{
public:
    DccPktSpeed128(int adrs = 3, int speed = 0);
    virtual int set_address(int adrs) override;
    int get_speed() const;
    void set_speed(int speed);
    static bool is_type(const uint8_t *msg, int msg_len);
    static uint8_t int_to_dcc(int speed_int);
    static int dcc_to_int(uint8_t speed_dcc);

private:
    void refresh(int adrs, int speed);
    DccPktSpeed128(const uint8_t *msg, int msg_len) : DccPkt(msg, msg_len) {}
    friend DccPkt create(const uint8_t *msg, int msg_len);
};

// 2.3.4 - Function Group One (F0-F4)
class DccPktFunc0 : public DccPkt
{
public:
    DccPktFunc0(int adrs = 3);
    virtual int set_address(int adrs) override;
    bool get_f(int num) const;
    void set_f(int num, bool on);
    static bool is_type(const uint8_t *msg, int msg_len);

private:
    static const int f_min = 0;
    static const int f_max = 4;
    void refresh(int adrs, uint8_t funcs = 0);
    DccPktFunc0(const uint8_t *msg, int msg_len) : DccPkt(msg, msg_len) {}
    uint8_t get_funcs() const;
    friend DccPkt create(const uint8_t *msg, int msg_len);
};

// 2.3.5 - Function Group Two (S-bit=1, F5-F8)
class DccPktFunc5 : public DccPkt
{
public:
    DccPktFunc5(int adrs = 3);
    virtual int set_address(int adrs) override;
    bool get_f(int num) const;
    void set_f(int num, bool on);
    static bool is_type(const uint8_t *msg, int msg_len);

private:
    static const int f_min = 5;
    static const int f_max = 8;
    void refresh(int adrs, uint8_t funcs = 0);
    DccPktFunc5(const uint8_t *msg, int msg_len) : DccPkt(msg, msg_len) {}
    uint8_t get_funcs() const;
    friend DccPkt create(const uint8_t *msg, int msg_len);
};

// 2.3.5 - Function Group Two (S-bit=0, F9-F12)
class DccPktFunc9 : public DccPkt
{
public:
    DccPktFunc9(int adrs = 3);
    virtual int set_address(int adrs) override;
    bool get_f(int num) const;
    void set_f(int num, bool on);
    static bool is_type(const uint8_t *msg, int msg_len);

private:
    static const int f_min = 9;
    static const int f_max = 12;
    void refresh(int adrs, uint8_t funcs = 0);
    DccPktFunc9(const uint8_t *msg, int msg_len) : DccPkt(msg, msg_len) {}
    uint8_t get_funcs() const;
    friend DccPkt create(const uint8_t *msg, int msg_len);
};

// 2.3.6.5 - F13-F20 Function Control
class DccPktFunc13 : public DccPkt
{
public:
    DccPktFunc13(int adrs = 3);
    virtual int set_address(int adrs) override;
    bool get_f(int num) const;
    void set_f(int num, bool on);
    static bool is_type(const uint8_t *msg, int msg_len);

private:
    static const int f_min = 13;
    static const int f_max = 20;
    void refresh(int adrs, uint8_t funcs = 0);
    DccPktFunc13(const uint8_t *msg, int msg_len) : DccPkt(msg, msg_len) {}
    uint8_t get_funcs() const;
    friend DccPkt create(const uint8_t *msg, int msg_len);
};

// 2.3.6.6 - F21-F28 Function Control
class DccPktFunc21 : public DccPkt
{
public:
    DccPktFunc21(int adrs = 3);
    virtual int set_address(int adrs) override;
    bool get_f(int num) const;
    void set_f(int num, bool on);
    static bool is_type(const uint8_t *msg, int msg_len);

private:
    static const int f_min = 21;
    static const int f_max = 28;
    void refresh(int adrs, uint8_t funcs = 0);
    DccPktFunc21(const uint8_t *msg, int msg_len) : DccPkt(msg, msg_len) {}
    uint8_t get_funcs() const;
    friend DccPkt create(const uint8_t *msg, int msg_len);
};

// 2.3.7.3 - Configuration Variable Access - Long Form (write byte)
class DccPktOpsWriteCv : public DccPkt
{
public:
    DccPktOpsWriteCv(int adrs = 3, int cv_num = 1, uint8_t cv_val = 0);
    virtual int set_address(int adrs) override;
    void set_cv(int cv_num, uint8_t cv_val);  // set in message
private:
    void refresh(int adrs, int cv_num, uint8_t cv_val);
    int get_cv_num() const;      // get from message
    uint8_t get_cv_val() const;  // get from message
};

// 2.3.7.3 - Configuration Variable Access - Long Form (bit manipulation)
class DccPktOpsWriteBit : public DccPkt
{
public:
    DccPktOpsWriteBit();
    DccPktOpsWriteBit(int adrs, int cv_num, int bit_num, int bit_val);
    virtual int set_address(int adrs) override;
    void set_cv_bit(int cv_num, int bit_num, int bit_val);

private:
    void refresh(int adrs, int cv_num, int bit_num, int bit_val);
    int get_cv_num() const;
    int get_bit_num() const;
    int get_bit_val() const;
};

// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcWriteCv : public DccPkt
{
public:
    DccPktSvcWriteCv(int cv_num = 1, uint8_t cv_val = 0);
    void set_cv(int cv_num, uint8_t cv_val);
};

// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcWriteBit : public DccPkt
{
public:
    DccPktSvcWriteBit(int cv_num = 1, int bit_num = 0, int bit_val = 0);
    void set_cv_bit(int cv_num, int bit_num, int bit_val);
};

// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcVerifyCv : public DccPkt
{
public:
    DccPktSvcVerifyCv(int cv_num = 1, uint8_t cv_val = 0);
    void set_cv_num(int cv_num);
    void set_cv_val(uint8_t cv_val);
};

// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcVerifyBit : public DccPkt
{
public:
    DccPktSvcVerifyBit(int cv_num = 1, int bit_num = 0, int bit_val = 0);
    void set_cv_num(int cv_num);
    void set_bit(int bit_num, int bit_val);
};

DccPkt create(const uint8_t *msg, int msg_len);