#include "dcc_pkt.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include "pico/types.h" // uint

#include "xassert.h"

#define PRINT_BRIEF 1

DccPkt::DccPkt(const uint8_t *msg, int msg_len)
{
    xassert(msg_len >= 0);

    memset(_msg, 0, msg_max);

    if (msg_len > msg_max) msg_len = 0;

    if (msg != nullptr) memcpy(_msg, msg, msg_len);

    _msg_len = msg_len;
}

void DccPkt::msg_len(int new_len)
{
    xassert(0 <= new_len && new_len <= msg_max);

    _msg_len = new_len;
}

uint8_t DccPkt::data(int idx) const
{
    xassert(0 <= idx && idx < _msg_len);

    return _msg[idx];
}

int DccPkt::get_address() const
{
    // absolute minimum is one byte of address and xor byte
    if (_msg_len < 2) return address_inv;

    const uint8_t b0 = _msg[0];

    if (b0 < 128) {
        // broadcast (0) or multi-function decoder with 7-bit address
        return b0;

    } else if (b0 < 192) {
        // 128-191: accessory decoder with 9- or 11-bit address
        // absolute minimum is now three bytes
        if (_msg_len < 3) return address_inv;
        const uint8_t b1 = _msg[1];
        int adrs = (int(b0 & 0x3f) << 2) | (int(~b1 & 0x70) << 4) |
                   (int(b1 & 0x06) >> 1);
        return adrs;

    } else if (b0 < 232) {
        // multi-function decoder with 14-bit address
        // absolute minimum is now three bytes
        if (_msg_len < 3) return address_inv;
        return (int(b0 & 0x3f) << 8) | _msg[1];

    } else if (b0 < 253) {
        // reserved
        return address_inv;

    } else if (b0 < 255) {
        // advanced extended packet
        return address_inv;

    } else {
        // idle packet (address = 255)
        return b0;
    }

}  // DccPkt::get_address

// set address in packet
// return address bytes used (1 or 2)
int DccPkt::set_address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    if (adrs <= address_short_max) {
        // one byte in packet
        xassert(msg_max >= 1);
        _msg[0] = adrs;
        return 1;
    } else {
        // two bytes in packet
        xassert(msg_max >= 2);
        _msg[0] = 0xc0 | ((adrs >> 8) & 0x3f);
        xassert(0xc0 <= _msg[0] && _msg[0] <= 0xe7);
        _msg[1] = adrs & 0xff;
        return 2;
    }
}

// get size of address
// return address bytes used (1 or 2)
int DccPkt::get_address_size() const
{
    if (_msg[0] <= address_short_max)
        return 1;
    else
        return 2;
}

void DccPkt::set_xor()
{
    xassert(_msg_len > 0);
    xassert(_msg_len < msg_max);

    uint8_t b = 0x00;
    for (int i = 0; i < (_msg_len - 1); i++) b ^= _msg[i];
    _msg[_msg_len - 1] = b;
}

bool DccPkt::check_xor() const
{
    uint8_t x = 0;
    for (int i = 0; i < _msg_len; i++) x ^= _msg[i];
    return x == 0;
}

bool DccPkt::check_xor(const uint8_t *msg, int msg_len)
{
    uint8_t x = 0;
    for (int i = 0; i < msg_len; i++) x ^= msg[i];
    return x == 0;
}

// Returns true if the current packet could be a service direct-mode packet
//
// Whether it's a service packet is state-dependent (i.e. decoder has been put
// in service mode). Decoding of the packet might be more state-dependent; it
// looks like there is overlap in the packet bit patterns:
//
// 0 0111_1101 0 0000_0001 0 EEEE_EEEE 1                Page preset
// 0 0111_0000 0 0DDD_DDDD 0 EEEE_EEEE 1                Address only verify
// 0 0111_1000 0 0DDD_DDDD 0 EEEE_EEEE 1                Address only write
// 0 0111_0RRR 0 DDDD_DDDD 0 EEEE_EEEE 1                Phys register verify
// 0 0111_1RRR 0 DDDD_DDDD 0 EEEE_EEEE 1                Phys register write
// 0 0111_0RRR 0 DDDD_DDDD 0 EEEE_EEEE 1                Page mode verify
// 0 0111_1RRR 0 DDDD_DDDD 0 EEEE_EEEE 1                Page mode write
//                                                          RRR: 000..101
// 0 0111_1111 0 0000_1000 0 EEEE_EEEE 1                Decoder factory reset
//
// 0 0111_01AA 0 AAAA_AAAA 0 DDDD_DDDD 0 EEEE_EEEE 1    Direct mode verify
// 0 0111_11AA 0 AAAA_AAAA 0 DDDD_DDDD 0 EEEE_EEEE 1    Direct mode write
// 0 0111_10AA 0 AAAA_AAAA 0 111K_DBBB 0 EEEE_EEEE 1    Direct mode bit manip

bool DccPkt::is_svc_direct(const uint8_t *msg, int msg_len)
{
    if (msg_len != 4) return false;

    // 1st byte without the two address bits
    const uint8_t b0 = msg[0] & 0xfc;

    // for write and verify, 2nd and 3rd bytes can be anything

    if (b0 == 0x74 || b0 == 0x7c) return true;  // write or verify

    // for bit manip, check those 1 bits in 3rd byte

    if (b0 == 0x78 && (msg[2] & 0xe0) == 0xe0) return true;  // bit manipulation

    return false;
}

bool DccPkt::decode_speed_128(int &speed) const
{
    // address (1 or 2 bytes), instruction, speed, xor
    if (_msg_len != 4 && _msg_len != 5) return false;
    int idx = get_address_size();
    if (_msg[idx] != 0x3f) return false;
    idx++;
    speed = DccPktSpeed128::dcc_to_int(_msg[idx]);
    return true;
}

bool DccPkt::decode_func_0(int *f) const  // f[5], f0..f4
{
    // address (1 or 2 bytes), instruction, xor
    if (_msg_len != 3 && _msg_len != 4) return false;
    int idx = get_address_size();
    uint8_t instr = _msg[idx];
    if ((instr & 0xe0) != 0x80) return false;
    f[0] = (instr >> 4) & 1;  // f0
    f[1] = (instr >> 0) & 1;  // f1
    f[2] = (instr >> 1) & 1;  // f2
    f[3] = (instr >> 2) & 1;  // f3
    f[4] = (instr >> 3) & 1;  // f4
    return true;
}

bool DccPkt::decode_func_5(int *f) const  // f[4], f5..f8
{
    // address (1 or 2 bytes), instruction, xor
    if (_msg_len != 3 && _msg_len != 4) return false;
    int idx = get_address_size();
    uint8_t instr = _msg[idx];
    if ((instr & 0xf0) != 0xb0) return false;
    f[0] = (instr >> 0) & 1;  // f5
    f[1] = (instr >> 1) & 1;  // f5
    f[2] = (instr >> 2) & 1;  // f6
    f[3] = (instr >> 3) & 1;  // f7
    return true;
}

bool DccPkt::decode_func_9(int *f) const  // f[4], f9..f12
{
    // address (1 or 2 bytes), instruction, xor
    if (_msg_len != 3 && _msg_len != 4) return false;
    int idx = get_address_size();
    uint8_t instr = _msg[idx];
    if ((instr & 0xf0) != 0xa0) return false;
    f[0] = (instr >> 0) & 1;  // f9
    f[1] = (instr >> 1) & 1;  // f10
    f[2] = (instr >> 2) & 1;  // f11
    f[3] = (instr >> 3) & 1;  // f12
    return true;
}

bool DccPkt::decode_func_13(int *f) const  // f[8], f13..f20
{
    // address (1 or 2 bytes), instruction, f-bits, xor
    if (_msg_len != 4 && _msg_len != 5) return false;
    int idx = get_address_size();
    uint8_t instr = _msg[idx];
    if (instr != DccPktFunc13::inst_byte) return false;
    idx++;
    uint8_t f_bits = _msg[idx];
    for (int b = 0; b < 8; b++) f[b] = (f_bits >> b) & 1;  // f13..f20
    return true;
}

#if INCLUDE_DCC_FUNC_21
bool DccPkt::decode_func_21(int *f) const  // f[8], f21..f28
{
    // address (1 or 2 bytes), instruction, f-bits, xor
    if (_msg_len != 4 && _msg_len != 5) return false;
    int idx = get_address_size();
    uint8_t instr = _msg[idx];
    if (instr != DccPktFunc21::inst_byte) return false;
    idx++;
    uint8_t f_bits = _msg[idx];
    for (int b = 0; b < 8; b++) f[b] = (f_bits >> b) & 1;  // f21..f28
    return true;
}
#endif

#if INCLUDE_DCC_FUNC_29
bool DccPkt::decode_func_29(int *f) const  // int f[8] is f29..f36
{
    // address (1 or 2 bytes), instruction, f-bits, xor
    if (_msg_len != 4 && _msg_len != 5) return false;
    int idx = get_address_size();
    uint8_t instr = _msg[idx];
    if (instr != DccPktFunc29::inst_byte) return false;
    idx++;
    uint8_t f_bits = _msg[idx];
    for (int b = 0; b < 8; b++) f[b] = (f_bits >> b) & 1;  // f29..f36
    return true;
}
#endif

char *DccPkt::dump(char *buf, int buf_len) const
{
    xassert(buf != nullptr);
    xassert(buf_len >= 0);

    memset(buf, '\0', buf_len);

    char *b = buf;
    char *e = buf + buf_len;

    b += snprintf(b, e - b, "{");

    if (e < b) return buf;

    for (int i = 0; i < _msg_len; i++) {
        b += snprintf(b, e - b, " %02x", _msg[i]);
        if (e < b) return buf;
    }

    b += snprintf(b, e - b, " }");

    return buf;
}

char *DccPkt::show(char *buf, int buf_len) const
{
    xassert(buf != nullptr);
    xassert(buf_len >= 0);

    memset(buf, '\0', buf_len);

    char *b = buf;
    char *e = buf + buf_len;

    b += snprintf(b, e - b, "D ");

    int idx = 0;

    if (!check_len_min(b, e, 2)) return buf;

    uint8_t b0 = _msg[idx++];
    xassert(idx == 1);

    if (b0 < 128 || (192 <= b0 && b0 < 232)) {
        int adrs = b0;

        // check for service mode packet
        if (is_svc_direct(_msg, _msg_len)) {
#if PRINT_BRIEF
            b += snprintf(b, e - b, "svc ");
#else
            b += snprintf(b, e - b, "  svc: ");
#endif
            // it's 4 bytes long with the correct constant bits
            show_cv_access(b, e, _msg[0], 1);
            return buf;
        } else if (b0 >= 128) {
            // long address
            if (!check_len_min(b, e, idx + 2)) return buf;
            uint8_t b1 = _msg[idx++];
            adrs = ((adrs & 0x3f) << 8) | b1;
        }

        // idx is now the index of the first byte after the address
        xassert(idx == 1 || idx == 2);

#if PRINT_BRIEF
        b += snprintf(b, e - b, "%d ", adrs);
#else
        b += snprintf(b, e - b, "%5d: ", adrs);
#endif

        if (!check_len_min(b, e, idx + 2)) return buf;

        uint8_t instr = _msg[idx++];

        if (instr == 0x00) {
            b += snprintf(b, e - b, "reset");

            if (!check_len_is(b, e, idx + 1)) return buf;

        } else if (instr == 0x3f) {
            if (!check_len_min(b, e, idx + 2)) return buf;

            int speed = _msg[idx++];

            if (speed & 0x80)
                b += snprintf(b, e - b, "+%d/128", speed & 0x7f);
            else
                b += snprintf(b, e - b, "-%d/128", speed & 0x7f);

            if (!check_len_is(b, e, idx + 1)) return buf;

        } else if ((instr & 0xe0) == 0x80) {
#if PRINT_BRIEF
            uint bits = ((instr & 0x0f) << 1) | ((instr & 0x10) >> 4);
            b += snprintf(b, e - b, "f0=%02x", bits);
#else
            b += snprintf(b, e - b, "f0%c f1%c f2%c f3%c f4%c",
                          instr & 0x10 ? '+' : '-', instr & 0x01 ? '+' : '-',
                          instr & 0x02 ? '+' : '-', instr & 0x04 ? '+' : '-',
                          instr & 0x08 ? '+' : '-');
#endif
            if (!check_len_is(b, e, idx + 1)) return buf;

        } else if ((instr & 0xf0) == 0xb0) {
#if PRINT_BRIEF
            b += snprintf(b, e - b, "f5=%02x", instr & 0x0f);
#else
            b += snprintf(b, e - b, "f5%c f6%c f7%c f8%c",
                          instr & 0x01 ? '+' : '-', instr & 0x02 ? '+' : '-',
                          instr & 0x04 ? '+' : '-', instr & 0x08 ? '+' : '-');
#endif
            if (!check_len_is(b, e, idx + 1)) return buf;

        } else if ((instr & 0xf0) == 0xa0) {
#if PRINT_BRIEF
            b += snprintf(b, e - b, "f9=%02x", instr & 0x0f);
#else
            b += snprintf(b, e - b, "f9%c f10%c f11%c f12%c",
                          instr & 0x01 ? '+' : '-', instr & 0x02 ? '+' : '-',
                          instr & 0x04 ? '+' : '-', instr & 0x08 ? '+' : '-');
#endif
            if (!check_len_is(b, e, idx + 1)) return buf;

        } else if ((instr & 0xf0) == 0xe0) {
            // ops mode cv access
            show_cv_access(b, e, instr, idx);

        } else if (instr == DccPktFunc13::inst_byte) {
            if (!check_len_min(b, e, idx + 2)) return buf;
#if PRINT_BRIEF
            b += snprintf(b, e - b, "f13=%02x", uint(_msg[idx++]));
#else
            uint8_t f = _msg[idx++];
            b += snprintf(b, e - b,
                          "f13%c f14%c f15%c f16%c f17%c f18%c f19%c f20%c",
                          f & 0x01 ? '+' : '-', f & 0x02 ? '+' : '-',
                          f & 0x04 ? '+' : '-', f & 0x08 ? '+' : '-',
                          f & 0x10 ? '+' : '-', f & 0x20 ? '+' : '-',
                          f & 0x40 ? '+' : '-', f & 0x80 ? '+' : '-');
#endif
            if (!check_len_is(b, e, idx + 1)) return buf;

#if INCLUDE_DCC_FUNC_21
        } else if (instr == DccPktFunc21::inst_byte) {
            if (!check_len_min(b, e, idx + 2)) return buf;
#if PRINT_BRIEF
            b += snprintf(b, e - b, "f21=%02x", uint(_msg[idx++]));
#else
            uint8_t f = _msg[idx++];
            b += snprintf(b, e - b,
                          "f21%c f22%c f23%c f24%c f25%c f26%c f27%c f28%c",
                          f & 0x01 ? '+' : '-', f & 0x02 ? '+' : '-',
                          f & 0x04 ? '+' : '-', f & 0x08 ? '+' : '-',
                          f & 0x10 ? '+' : '-', f & 0x20 ? '+' : '-',
                          f & 0x40 ? '+' : '-', f & 0x80 ? '+' : '-');
#endif
            if (!check_len_is(b, e, idx + 1)) return buf;
#endif // INCLUDE_DCC_FUNC_21

#if INCLUDE_DCC_FUNC_29
        } else if (instr == DccPktFunc29::inst_byte) {
            if (!check_len_min(b, e, idx + 2)) return buf;
#if PRINT_BRIEF
            b += snprintf(b, e - b, "f29=%02x", uint(_msg[idx++]));
#else
            uint8_t f = _msg[idx++];
            b += snprintf(b, e - b,
                          "f29%c f30%c f31%c f32%c f33%c f34%c f35%c f36%c",
                          f & 0x01 ? '+' : '-', f & 0x02 ? '+' : '-',
                          f & 0x04 ? '+' : '-', f & 0x08 ? '+' : '-',
                          f & 0x10 ? '+' : '-', f & 0x20 ? '+' : '-',
                          f & 0x40 ? '+' : '-', f & 0x80 ? '+' : '-');
#endif
            if (!check_len_is(b, e, idx + 1)) return buf;
#endif // INCLUDE_DCC_FUNC_29

#if INCLUDE_DCC_FUNC_37
        } else if (instr == DccPktFunc37::inst_byte) {
            if (!check_len_min(b, e, idx + 2)) return buf;
#if PRINT_BRIEF
            b += snprintf(b, e - b, "f37=%02x", uint(_msg[idx++]));
#else
            uint8_t f = _msg[idx++];
            b += snprintf(b, e - b,
                          "f37%c f38%c f39%c f40%c f41%c f42%c f43%c f44%c",
                          f & 0x01 ? '+' : '-', f & 0x02 ? '+' : '-',
                          f & 0x04 ? '+' : '-', f & 0x08 ? '+' : '-',
                          f & 0x10 ? '+' : '-', f & 0x20 ? '+' : '-',
                          f & 0x40 ? '+' : '-', f & 0x80 ? '+' : '-');
#endif
            if (!check_len_is(b, e, idx + 1)) return buf;
#endif // INCLUDE_DCC_FUNC_37

#if INCLUDE_DCC_FUNC_45
        } else if (instr == DccPktFunc45::inst_byte) {
            if (!check_len_min(b, e, idx + 2)) return buf;
#if PRINT_BRIEF
            b += snprintf(b, e - b, "f45=%02x", uint(_msg[idx++]));
#else
            uint8_t f = _msg[idx++];
            b += snprintf(b, e - b,
                          "f45%c f46%c f47%c f48%c f49%c f50%c f51%c f52%c",
                          f & 0x01 ? '+' : '-', f & 0x02 ? '+' : '-',
                          f & 0x04 ? '+' : '-', f & 0x08 ? '+' : '-',
                          f & 0x10 ? '+' : '-', f & 0x20 ? '+' : '-',
                          f & 0x40 ? '+' : '-', f & 0x80 ? '+' : '-');
#endif
            if (!check_len_is(b, e, idx + 1)) return buf;
#endif // INCLUDE_DCC_FUNC_45

#if INCLUDE_DCC_FUNC_53
        } else if (instr == DccPktFunc53::inst_byte) {
            if (!check_len_min(b, e, idx + 2)) return buf;
#if PRINT_BRIEF
            b += snprintf(b, e - b, "f53=%02x", uint(_msg[idx++]));
#else
            uint8_t f = _msg[idx++];
            b += snprintf(b, e - b,
                          "f53%c f54%c f55%c f56%c f57%c f58%c f59%c f60%c",
                          f & 0x01 ? '+' : '-', f & 0x02 ? '+' : '-',
                          f & 0x04 ? '+' : '-', f & 0x08 ? '+' : '-',
                          f & 0x10 ? '+' : '-', f & 0x20 ? '+' : '-',
                          f & 0x40 ? '+' : '-', f & 0x80 ? '+' : '-');
#endif
            if (!check_len_is(b, e, idx + 1)) return buf;
#endif // INCLUDE_DCC_FUNC_53

#if INCLUDE_DCC_FUNC_61
        } else if (instr == DccPktFunc61::inst_byte) {
            if (!check_len_min(b, e, idx + 2)) return buf;
#if PRINT_BRIEF
            b += snprintf(b, e - b, "f61=%02x", uint(_msg[idx++]));
#else
            uint8_t f = _msg[idx++];
            b += snprintf(b, e - b,
                          "f61%c f62%c f63%c f64%c f65%c f66%c f67%c f68%c",
                          f & 0x01 ? '+' : '-', f & 0x02 ? '+' : '-',
                          f & 0x04 ? '+' : '-', f & 0x08 ? '+' : '-',
                          f & 0x10 ? '+' : '-', f & 0x20 ? '+' : '-',
                          f & 0x40 ? '+' : '-', f & 0x80 ? '+' : '-');
#endif
            if (!check_len_is(b, e, idx + 1)) return buf;
#endif // INCLUDE_DCC_FUNC_61

        }

    } else if (128 <= b0 && b0 < 192) {
        xassert(idx == 1);

        // 2.4.1 Basic Accessory Decoder Packet Format
        // [preamble] 0 10AAAAAA 0 1AAADAAR 0 EEEEEEEE 1
        // Packet length == 3

        // 2.4.3.1 Basic Accessory Decoder Operations Mode Programming
        // [preamble] 0 10AAAAAA 0 1AAA1AA0 0 1110GGVV 0 VVVVVVVV 0 DDDDDDDD 0
        // EEEEEEEE 1 Packet length == 6; maybe 5 if DDDDDDDD not required?

        // 2.4.4 Basic Accessory Decoder XPOM
        // [preamble] 0 10AAAAAA 0 1AAA1AA0 0 1110GGSS 0
        //              VVVVVVVV 0 VVVVVVVV 0 VVVVVVVV 0
        //              { DDDDDDDD 0 { DDDDDDDD 0 { DDDDDDDD 0 { DDDDDDDD 0 }}}}
        //              EEEEEEEE 1
        // Packet length == 7, 8, 9, 10, or 11

        // 2.4.2 Extended Accessory Decoder Control Packet Format
        // [preamble] 0 10AAAAAA 0 0AAA0AA1 0 XXXXXXXX 0 EEEEEEEE 1
        // Packet length == 4

        // 2.4.3.2 Extended Accessory Decoder Operations Mode Programming
        // [preamble] 0 10AAAAAA 0 0AAA0AA1 0 1110GGVV 0 VVVVVVVV 0 DDDDDDDD 0
        // EEEEEEEE 1 Packet length == 6; maybe 5 if DDDDDDDD not required?

        // 2.4.5 Extended Accessory Decoder XPOM
        // [preamble] 0 10AAAAAA 0 0AAA0AA1 0 1110GGSS 0
        //              VVVVVVVV 0 VVVVVVVV 0 VVVVVVVV 0
        //              { DDDDDDDD 0 { DDDDDDDD 0 { DDDDDDDD 0 { DDDDDDDD 0 }}}}
        //              EEEEEEEE 1
        // Zero to four DDDDDDDD bytes
        // Packet length == 7, 8, 9, 10, or 11

        // 2.4.6 NOP
        // [preamble] 0 10AAAAAA 0 0AAA1AAT 0 EEEEEEEE 1
        // Packet length = 3

        if (!check_len_min(b, e, 3)) return buf;

        uint8_t b1 = _msg[1];
        int adrs = (int(b0 & 0x3f) << 2) | (int(~b1 & 0x70) << 4) |
                   (int(b1 & 0x06) >> 1);

        int m = (b1 >> 7) & 1;
        int d = (b1 >> 3) & 1;
        int r = (b1 >> 0) & 1;

        b += snprintf(b, e - b, "%5d: acc m=%d d=%d r=%d: ", adrs, m, d, r);

        dump(b, e - b);

    } else if (b0 == 255) {
#if PRINT_BRIEF
        b += snprintf(b, e - b, "idle");
#else
        b += snprintf(b, e - b, "       idle");
#endif

    } else {
        // "reserved" (232-252) or "advanced extended" (253-254)
        dump(b, e - b);

    }  // if (b0...)

    return buf;

}  // DccPkt::show

//----------------------------------------------------------------------------

bool DccPkt::check_len_min(char *&b, char *e, int min_len) const
{
    if (_msg_len >= min_len) return true;

    b += snprintf(b, e - b, "(short packet)");
    dump(b, e - b);
    return false;
}

//----------------------------------------------------------------------------

bool DccPkt::check_len_is(char *&b, char *e, int len) const
{
    if (_msg_len == len) return true;

    b += snprintf(b, e - b, " (unexpected length)");
    dump(b, e - b);
    return false;
}

//----------------------------------------------------------------------------

void DccPkt::show_cv_access(char *&b, char *e, uint8_t instr, int idx) const
{
    // svc mode: instr is 0111_GGAA
    // ops mode: instr is 1110_GGAA
    // GG is the operation (1, 2, or 3)
    // AA is bits 8 and 9 of the cv number
    xassert((instr & 0xf0) == 0x70 || (instr & 0xf0) == 0xe0);

    int op = (instr & 0x0c) >> 2;  // 1, 2, or 3
    int cv = instr & 0x03;
    cv = (cv << 8) | _msg[idx];
    idx++;
    cv++;  // by convention, cv number starts at 1

    uint8_t data = _msg[idx];
    idx++;

    if (op == 0) {
        // reserved
#if PRINT_BRIEF
        b += snprintf(b, e - b, "op=%d!", op);
#else
        b += snprintf(b, e - b, "op=%d! (reserved)", op);
#endif
    } else if (op == 1) {
        // verify byte (expected in svc mode only)
#if PRINT_BRIEF
        b += snprintf(b, e - b, "cv%d=0x%02x?", cv, data);
#else
        b += snprintf(b, e - b, "verify cv%d=0x%02x", cv, data);
#endif
    } else if (op == 2) {
        // bit manipulation
        int bit = data & 0x07;         // 0..7
        int val = (data & 0x08) >> 3;  // 0..1
        if (data & 0x10) {
            // write bit
#if PRINT_BRIEF
            b += snprintf(b, e - b, "cv%d[%d]=%d", cv, bit, val);
#else
            b += snprintf(b, e - b, "write cv%d bit%d=%d", cv, bit, val);
#endif
        } else {
            // verify bit (expected in svc mode only)
#if PRINT_BRIEF
            b += snprintf(b, e - b, "cv%d[%d]=%d?", cv, bit, val);
#else
            b += snprintf(b, e - b, "verify cv%d bit%d=%d", cv, bit, val);
#endif
        }
    } else {  // op == 3
        // write byte
#if PRINT_BRIEF
        b += snprintf(b, e - b, "cv%d=0x%02x", cv, data);
#else
        b += snprintf(b, e - b, "write cv%d=0x%02x", cv, data);
#endif
    }

    (void)check_len_is(b, e, idx + 1);

}  // DccPkt::show_cv_access

//----------------------------------------------------------------------------

DccPktIdle::DccPktIdle()
{
    _msg[0] = 0xff;
    _msg[1] = 0x00;
    _msg_len = 3;
    set_xor();
}

//----------------------------------------------------------------------------

DccPktReset::DccPktReset()
{
    _msg[0] = 0x00;
    _msg[1] = 0x00;
    _msg_len = 3;
    set_xor();
}

//----------------------------------------------------------------------------

DccPktSpeed128::DccPktSpeed128(int adrs, int speed)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(speed_min <= speed && speed <= speed_max);

    refresh(adrs, speed);
}

bool DccPktSpeed128::is_type(const uint8_t *msg, int msg_len)
{
    if (msg_len < 1) return false;
    uint8_t b0 = msg[0];
    if (1 <= b0 && b0 <= 127)
        return msg_len == 4 && msg[1] == 0x3f &&
               DccPkt::check_xor(msg, msg_len);
    else if (192 <= b0 && b0 <= 231)
        return msg_len == 5 && msg[2] == 0x3f &&
               DccPkt::check_xor(msg, msg_len);
    else
        return false;
}

int DccPktSpeed128::set_address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, get_speed());
    return get_address_size();
}

int DccPktSpeed128::get_speed() const
{
    int idx = get_address_size() + 1;  // skip address and inst byte (0x3f)
    return dcc_to_int(_msg[idx]);
}

void DccPktSpeed128::set_speed(int speed)
{
    xassert(speed_min <= speed && speed <= speed_max);

    int idx = get_address_size() + 1;  // skip address and inst byte (0x3f)
    _msg[idx] = int_to_dcc(speed);
    set_xor();
}

void DccPktSpeed128::refresh(int adrs, int speed)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(speed_min <= speed && speed <= speed_max);

    int idx = DccPkt::set_address(adrs);  // 1 or 2 bytes
    _msg[idx++] = 0x3f;                   // CCC=001 GGGGG=11111
    _msg[idx++] = int_to_dcc(speed);
    _msg_len = idx + 1;  // 4 or 5
    set_xor();
}

// DCC speed: msb: 1 is forward, 0 is reverse, remaining bits are magnitude

uint8_t DccPktSpeed128::int_to_dcc(int speed_int)
{
    if (speed_int < 0)
        return -speed_int;
    else
        return speed_int | 0x80;
}

int DccPktSpeed128::dcc_to_int(uint8_t speed_dcc)
{
    if (speed_dcc & 0x80)
        return speed_dcc & ~0x80;  // forward, just clear msb
    else
        return -int(speed_dcc);  // reverse, return negative of magnitude
}

//----------------------------------------------------------------------------

DccPktFunc0::DccPktFunc0(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs);
}

int DccPktFunc0::set_address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, get_funcs());
    return get_address_size();
}

bool DccPktFunc0::get_f(int num) const
{
    xassert(f_min <= num && num <= f_max);

    int idx = get_address_size();  // skip address
    uint8_t f_bit =
        ((num == 0) ? 0x10 : (0x01 << (num - 1)));  // bit for function
    return (_msg[idx] & f_bit) != 0;
}

void DccPktFunc0::set_f(int num, bool on)
{
    xassert(f_min <= num && num <= f_max);

    int idx = get_address_size();  // skip address
    uint8_t f_bit =
        ((num == 0) ? 0x10 : (0x01 << (num - 1)));  // bit for function
    if (on)
        _msg[idx] |= f_bit;
    else
        _msg[idx] &= ~f_bit;
    set_xor();
}

bool DccPktFunc0::is_type(const uint8_t *msg, int msg_len)
{
    if (msg_len < 1) return false;
    uint8_t b0 = msg[0];
    if (1 <= b0 && b0 <= 127)
        return msg_len == 3 && (msg[1] & 0xe0) == 0x80 &&
               DccPkt::check_xor(msg, msg_len);
    else if (192 <= b0 && b0 <= 231)
        return msg_len == 4 && (msg[2] & 0xe0) == 0x80 &&
               DccPkt::check_xor(msg, msg_len);
    else
        return false;
}

void DccPktFunc0::refresh(int adrs, uint8_t funcs)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert((funcs & ~0x1f) == 0);

    int idx = DccPkt::set_address(adrs);  // 1 or 2 bytes
    _msg[idx++] = 0x80 | funcs;           // CCC=100, then f0:f4:f3:f2:f1
    _msg_len = idx + 1;                   // 3 or 4
    set_xor();
}

uint8_t DccPktFunc0::get_funcs() const
{
    int idx = get_address_size();  // skip address
    return _msg[idx] & 0x1f;       // lower 5 bits
}

//----------------------------------------------------------------------------

DccPktFunc5::DccPktFunc5(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs);
}

int DccPktFunc5::set_address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, get_funcs());
    return get_address_size();
}

bool DccPktFunc5::get_f(int num) const
{
    xassert(f_min <= num && num <= f_max);

    int idx = get_address_size();           // skip address
    uint8_t f_bit = 0x01 << (num - f_min);  // bit for function
    return (_msg[idx] & f_bit) != 0;
}

void DccPktFunc5::set_f(int num, bool on)
{
    xassert(f_min <= num && num <= f_max);

    int idx = get_address_size();           // skip address
    uint8_t f_bit = 0x01 << (num - f_min);  // bit for function
    if (on)
        _msg[idx] |= f_bit;
    else
        _msg[idx] &= ~f_bit;
    set_xor();
}

bool DccPktFunc5::is_type(const uint8_t *msg, int msg_len)
{
    if (msg_len < 1) return false;
    uint8_t b0 = msg[0];
    if (1 <= b0 && b0 <= 127)
        return msg_len == 3 && (msg[1] & 0xf0) == 0xb0 &&
               DccPkt::check_xor(msg, msg_len);
    else if (192 <= b0 && b0 <= 231)
        return msg_len == 4 && (msg[2] & 0xf0) == 0xb0 &&
               DccPkt::check_xor(msg, msg_len);
    else
        return false;
}

void DccPktFunc5::refresh(int adrs, uint8_t funcs)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert((funcs & ~0x0f) == 0);

    int idx = DccPkt::set_address(adrs);  // 1 or 2 bytes
    _msg[idx++] = 0xb0 | funcs;           // CCC=101, S=1, then f8:f7:f6:f5
    _msg_len = idx + 1;                   // 3 or 4
    set_xor();
}

uint8_t DccPktFunc5::get_funcs() const
{
    int idx = get_address_size();  // skip address
    return _msg[idx] & 0x0f;       // lower 4 bits
}

//----------------------------------------------------------------------------

DccPktFunc9::DccPktFunc9(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs);
}

int DccPktFunc9::set_address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, get_funcs());
    return get_address_size();
}

bool DccPktFunc9::get_f(int num) const
{
    xassert(f_min <= num && num <= f_max);

    int idx = get_address_size();           // skip address
    uint8_t f_bit = 0x01 << (num - f_min);  // bit for function
    return (_msg[idx] & f_bit) != 0;
}

void DccPktFunc9::set_f(int num, bool on)
{
    xassert(f_min <= num && num <= f_max);

    int idx = get_address_size();           // skip address
    uint8_t f_bit = 0x01 << (num - f_min);  // bit for function
    if (on)
        _msg[idx] |= f_bit;
    else
        _msg[idx] &= ~f_bit;
    set_xor();
}

bool DccPktFunc9::is_type(const uint8_t *msg, int msg_len)
{
    if (msg_len < 1) return false;
    uint8_t b0 = msg[0];
    if (1 <= b0 && b0 <= 127)
        return msg_len == 3 && (msg[1] & 0xf0) == 0xa0 &&
               DccPkt::check_xor(msg, msg_len);
    else if (192 <= b0 && b0 <= 231)
        return msg_len == 4 && (msg[2] & 0xf0) == 0xa0 &&
               DccPkt::check_xor(msg, msg_len);
    else
        return false;
}

void DccPktFunc9::refresh(int adrs, uint8_t funcs)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert((funcs & ~0x0f) == 0);

    int idx = DccPkt::set_address(adrs);  // 1 or 2 bytes
    _msg[idx++] = 0xa0 | funcs;           // CCC=101, S=0, then f12:f11:f10:f9
    _msg_len = idx + 1;                   // 3 or 4
    set_xor();
}

uint8_t DccPktFunc9::get_funcs() const
{
    int idx = get_address_size();  // skip address
    return _msg[idx] & 0x0f;       // lower 4 bits
}

//----------------------------------------------------------------------------

DccPktOpsReadCv::DccPktOpsReadCv(int adrs, int cv_num, uint8_t cv_val)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);

    refresh(adrs, cv_num, cv_val);
}

int DccPktOpsReadCv::set_address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, get_cv_num(), get_cv_val());
    return get_address_size();
}

void DccPktOpsReadCv::set_cv(int cv_num, uint8_t cv_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024

    cv_num--;                      // cv_num is encoded in messages as 0..1023
    int idx = get_address_size();  // skip address (1 or 2 bytes)
    _msg[idx++] = 0xec | (cv_num >> 8);  // 111011vv
    _msg[idx++] = cv_num;                // vvvvvvvv
    _msg[idx++] = cv_val;                // dddddddd
    _msg_len = idx + 1;                  // total (with xor) 5 or 6 bytes
    set_xor();
}

void DccPktOpsReadCv::refresh(int adrs, int cv_num, uint8_t cv_val)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024

    (void)DccPkt::set_address(adrs);  // insert address (1 or 2 bytes)
    set_cv(cv_num, cv_val);           // insert everything else
}

int DccPktOpsReadCv::get_cv_num() const
{
    int idx = get_address_size();           // skip address (1 or 2 bytes)
    int cv_hi = _msg[idx++] & 0x03;         // get 2 hi bits
    int cv_num = (cv_hi << 8) | _msg[idx];  // get 8 lo bits
    return cv_num + 1;  // cv_num is 0..1023 in message, return 1..1024
}

uint8_t DccPktOpsReadCv::get_cv_val() const
{
    int idx = get_address_size() + 2;  // skip address, instruction, cv_num
    return _msg[idx];
}

//----------------------------------------------------------------------------

DccPktOpsWriteCv::DccPktOpsWriteCv(int adrs, int cv_num, uint8_t cv_val)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);

    refresh(adrs, cv_num, cv_val);
}

int DccPktOpsWriteCv::set_address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, get_cv_num(), get_cv_val());
    return get_address_size();
}

void DccPktOpsWriteCv::set_cv(int cv_num, uint8_t cv_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024

    cv_num--;                      // cv_num is encoded in messages as 0..1023
    int idx = get_address_size();  // skip address (1 or 2 bytes)
    _msg[idx++] = 0xec | (cv_num >> 8);  // 111011vv
    _msg[idx++] = cv_num;                // vvvvvvvv
    _msg[idx++] = cv_val;                // dddddddd
    _msg_len = idx + 1;                  // total (with xor) 5 or 6 bytes
    set_xor();
}

void DccPktOpsWriteCv::refresh(int adrs, int cv_num, uint8_t cv_val)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024

    (void)DccPkt::set_address(adrs);  // insert address (1 or 2 bytes)
    set_cv(cv_num, cv_val);           // insert everything else
}

int DccPktOpsWriteCv::get_cv_num() const
{
    int idx = get_address_size();           // skip address (1 or 2 bytes)
    int cv_hi = _msg[idx++] & 0x03;         // get 2 hi bits
    int cv_num = (cv_hi << 8) | _msg[idx];  // get 8 lo bits
    return cv_num + 1;  // cv_num is 0..1023 in message, return 1..1024
}

uint8_t DccPktOpsWriteCv::get_cv_val() const
{
    int idx = get_address_size() + 2;  // skip address, instruction, cv_num
    return _msg[idx];
}

//----------------------------------------------------------------------------

DccPktOpsWriteBit::DccPktOpsWriteBit(int adrs, int cv_num, int bit_num,
                                     int bit_val)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    refresh(adrs, cv_num, bit_num, bit_val);
}

// constructor for when we know we'll be setting fields later
DccPktOpsWriteBit::DccPktOpsWriteBit() { refresh(3, 8, 0, 0); }

int DccPktOpsWriteBit::set_address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, get_cv_num(), get_bit_num(), get_bit_val());
    return get_address_size();
}

// set cv_num, bit_num, and bit_val in message
void DccPktOpsWriteBit::set_cv_bit(int cv_num, int bit_num, int bit_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    cv_num--;                      // cv_num is encoded in messages as 0..1023
    int idx = get_address_size();  // skip address (1 or 2 bytes)
    _msg[idx++] = 0xe8 | (cv_num >> 8);             // 111010vv
    _msg[idx++] = cv_num;                           // vvvvvvvv
    _msg[idx++] = 0xf0 | (bit_val << 3) | bit_num;  // dddddddd
    _msg_len = idx + 1;  // total (with xor) 5 or 6 bytes
    set_xor();
}

// update message where bytes in address (1 or 2) might be changing
void DccPktOpsWriteBit::refresh(int adrs, int cv_num, int bit_num, int bit_val)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    (void)DccPkt::set_address(adrs);       // insert address (1 or 2 bytes)
    set_cv_bit(cv_num, bit_num, bit_val);  // insert everything else
}

// get cv_num from message
int DccPktOpsWriteBit::get_cv_num() const
{
    int idx = get_address_size();           // skip address (1 or 2 bytes)
    int cv_hi = _msg[idx++] & 0x03;         // get 2 hi bits
    int cv_num = (cv_hi << 8) | _msg[idx];  // get 8 lo bits
    return cv_num + 1;  // cv_num is 0..1023 in message, return 1..1024
}

// get bit_num from message
int DccPktOpsWriteBit::get_bit_num() const
{
    int idx = get_address_size() + 2;  // skip address, instruction, cv_num
    return _msg[idx] & 0x07;           // return lo 3 bits
}

// get bit_val from message
int DccPktOpsWriteBit::get_bit_val() const
{
    int idx = get_address_size() + 2;  // skip address, instruction, cv_num
    return (_msg[idx] >> 3) & 1;       // return bit 3
}

//----------------------------------------------------------------------------

DccPktSvcWriteCv::DccPktSvcWriteCv(int cv_num, uint8_t cv_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024

    set_cv(cv_num, cv_val);
}

void DccPktSvcWriteCv::set_cv(int cv_num, uint8_t cv_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024

    cv_num--;                        // cv_num is encoded in messages as 0..1023
    _msg[0] = 0x7c | (cv_num >> 8);  // 0111CCAA, CC=11 "write byte"
    _msg[1] = cv_num;                // AAAAAAAA
    _msg[2] = cv_val;                // DDDDDDDD
    _msg_len = 4;                    // total (with xor) 4 bytes
    set_xor();
}

//----------------------------------------------------------------------------

DccPktSvcWriteBit::DccPktSvcWriteBit(int cv_num, int bit_num, int bit_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024

    set_cv_bit(cv_num, bit_num, bit_val);
}

void DccPktSvcWriteBit::set_cv_bit(int cv_num, int bit_num, int bit_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    cv_num--;  // cv_num is encoded in messages as 0..1023
    _msg[0] = 0x78 | (cv_num >> 8);
    _msg[1] = cv_num;
    _msg[2] = 0xf0 | (bit_val << 3) | bit_num;
    _msg_len = 4;
    set_xor();
}

//----------------------------------------------------------------------------

DccPktSvcVerifyCv::DccPktSvcVerifyCv(int cv_num, uint8_t cv_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024

    set_cv_num(cv_num);
    set_cv_val(cv_val);
}

void DccPktSvcVerifyCv::set_cv_num(int cv_num)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024

    cv_num--;                        // cv_num is encoded in messages as 0..1023
    _msg[0] = 0x74 | (cv_num >> 8);  // 0111CCAA, CC=01 "verify byte"
    _msg[1] = cv_num;                // AAAAAAAA
    //_msg[2] = cv_val;               // DDDDDDDD
    _msg_len = 4;  // total (with xor) 4 bytes
    set_xor();
}

void DccPktSvcVerifyCv::set_cv_val(uint8_t cv_val)
{
    //_msg[0] = 0x74 | (cv_num >> 8); // 0111CCAA, CC=01 "verify byte"
    //_msg[1] = cv_num;               // AAAAAAAA
    _msg[2] = cv_val;  // DDDDDDDD
    _msg_len = 4;      // total (with xor) 4 bytes
    set_xor();
}

//----------------------------------------------------------------------------

DccPktSvcVerifyBit::DccPktSvcVerifyBit(int cv_num, int bit_num, int bit_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    set_cv_num(cv_num);
    set_bit(bit_num, bit_val);
}

void DccPktSvcVerifyBit::set_cv_num(int cv_num)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);  // 1..1024

    cv_num--;  // cv_num is encoded in messages as 0..1023
    _msg[0] = 0x78 | (cv_num >> 8);
    _msg[1] = cv_num;
    _msg[2] = 0xe0;  // bit_val=0, bit_num=0 (set later)
    _msg_len = 4;
    set_xor();
}

void DccPktSvcVerifyBit::set_bit(int bit_num, int bit_val)
{
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    // do not clobber cv_num
    //_msg[0] = 0x78 | (cv_num >> 8);
    //_msg[1] = cv_num;
    _msg[2] = 0xe0 | (bit_val << 3) | bit_num;
    //_msg_len = 4;
    set_xor();
}

// convert a received byte array into the correct message type
DccPkt create(const uint8_t *msg, int msg_len)
{
    DccPkt::PktType pkt_type = DccPkt::decode_type(msg, msg_len);

    switch (pkt_type) {
        case DccPkt::PktType::Reset:
            return DccPktReset();
        case DccPkt::PktType::Speed128:
            return DccPktSpeed128(msg, msg_len);
        case DccPkt::PktType::Func0:
            return DccPktFunc0(msg, msg_len);
        case DccPkt::PktType::Func5:
            return DccPktFunc5(msg, msg_len);

        case DccPkt::PktType::Idle:
            return DccPktIdle();
        default:
            return DccPkt(msg, msg_len);
    }
}

DccPkt::PktType DccPkt::decode_type(const uint8_t *msg, int msg_len)
{
    if (msg_len < 3) return Invalid;

    if (!check_xor(msg, msg_len)) return Invalid;

    uint8_t b0 = msg[0];
    if (b0 == 0) {
        if (msg_len == 3 && msg[1] == 0 && msg[2] == 0)
            return Reset;
        else
            return Invalid;
    } else if (b0 <= 127) {
        // 1..127: multifunction decoder with 7-bit address
        return decode_payload(msg + 1, msg_len - 1);
    } else if (b0 <= 191) {
        // 128..191: accessory decoder (basic or extended)
        return Accessory;
    } else if (b0 <= 231) {
        // 192..231: multifunction decoder with 14-bit address
        return decode_payload(msg + 2, msg_len - 2);
    } else if (b0 <= 252) {
        // 232..252: reserved
        return Reserved;
    } else if (b0 <= 254) {
        // 253..254: advanced extended
        return Advanced;
    } else {
        // 255
        if (msg_len == 3 && msg[1] == 0 && msg[2] == 0xff)
            return Idle;
        else
            return Invalid;
    }
}

DccPkt::PktType DccPkt::decode_payload(const uint8_t *pay, int pay_len)
{
    xassert(pay_len >= 1);
    uint8_t ccc = (pay[0] >> 5) & 0x07;
    if (ccc == 0) {
        // 2.3.1 Decoder and Consist Control
        return Unimplemented;
    } else if (ccc == 1) {
        // 2.3.2 Advanced Operations
        if (pay[0] == 0x3f && pay_len == 3) {
            return Speed128;
        } else {
            return Invalid;
        }
    } else if (ccc == 2 || ccc == 3) {
        // 2.3.3 Speed and Direction
        if (pay_len == 2) {
            return Speed28;
        } else {
            return Invalid;
        }
    } else if (ccc == 4) {
        // 2.3.4 Function Group 1
        if (pay_len == 2) {
            return Func0;  // F0..F4
        } else {
            return Invalid;
        }
    } else if (ccc == 5) {
        // 2.3.5 Function Group 2
        if (pay_len == 2) {
            if ((pay[0] & 0x10) != 0) {
                return Func5;  // F5..F8
            } else {
                return Func9;  // F9..F12
            }
        } else {
            return Invalid;
        }
    } else if (ccc == 6) {
        // 2.3.6 Feature Expansion
        uint8_t ggggg = pay[0] & 0x1f;
        if (ggggg == (DccPktFunc13::inst_byte & 0x1f) && pay_len == 3) {
            return Func13;
#if INCLUDE_DCC_FUNC_21
        } else if (ggggg == (DccPktFunc21::inst_byte & 0x1f) && pay_len == 3) {
            return Func21;
#endif
#if INCLUDE_DCC_FUNC_29
        } else if (ggggg == (DccPktFunc29::inst_byte & 0x1f) && pay_len == 3) {
            return Func29;
#endif
#if INCLUDE_DCC_FUNC_37
        } else if (ggggg == (DccPktFunc37::inst_byte & 0x1f) && pay_len == 3) {
            return Func37;
#endif
#if INCLUDE_DCC_FUNC_45
        } else if (ggggg == (DccPktFunc45::inst_byte & 0x1f) && pay_len == 3) {
            return Func45;
#endif
#if INCLUDE_DCC_FUNC_53
        } else if (ggggg == (DccPktFunc53::inst_byte & 0x1f) && pay_len == 3) {
            return Func53;
#endif
#if INCLUDE_DCC_FUNC_61
        } else if (ggggg == (DccPktFunc61::inst_byte & 0x1f) && pay_len == 3) {
            return Func61;
#endif
        } else {
            return Unimplemented;
        }
    } else {  // (ccc == 7)
        // 2.3.7 Configuration Variable Access
        uint8_t p0 = pay[0];
        if ((p0 & 0x10) == 0x10) {
            return Unimplemented;  // Short form
        } else {
            if (pay_len == 4) {
                // Long form
                uint8_t gg = (p0 >> 2) & 0x3;
                if (gg == 0) {
                    return Reserved;
                } else if (gg == 1) {
                    return Unimplemented;
                } else if (gg == 2) {
                    return OpsWriteBit;
                } else {  // (gg == 3)
                    return OpsWriteCv;
                }
            } else {
                return Unimplemented;  // xpom
            }
        }
    }
}  // DccPkt::PktType DccPkt::decode_payload(...)