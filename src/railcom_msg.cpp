
#include "railcom_msg.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "xassert.h"

// Extract one message from decoded 6-bit data
// Return true if message extracted, false on error
// Update d to point to next unused data


// channel 1 messages
bool RailComMsg::parse1(const uint8_t *&d, const uint8_t *d_end)
{
    int len = d_end - d; // 6-bit datum available
    if (len < 1) {
        return false;
    }

    uint8_t b0 = d[0];
    if (b0 < RailComSpec::DecId::dec_max) {
        // b0 was successfully decoded to 6 bits of data
        RailComSpec::PktId pkt_id = RailComSpec::PktId((b0 >> 2) & 0x0f);
        if (pkt_id == RailComSpec::PktId::pkt_ahi) {
            // 12 bit (2 byte) message
            if (len >= 2) {
                id = MsgId::ahi;
                ahi.ahi = ((b0 << 6) | d[1]) & 0xff;
                d += 2;
                return true;
            }
        } else if (pkt_id == RailComSpec::PktId::pkt_alo) {
            // 12 bit (2 byte) message
            if (len >= 2) {
                id = MsgId::alo;
                alo.alo = ((b0 << 6) | d[1]) & 0xff;
                d += 2;
                return true;
            }
        }
    }
    return false;
}


// channel 2 messages
bool RailComMsg::parse2(const uint8_t *&d, const uint8_t *d_end)
{
    int len = d_end - d; // 6-bit datum available
    if (len < 1) {
        return false;
    }

    uint8_t b0 = d[0];
    if (b0 == RailComSpec::DecId::dec_ack) {
        id = MsgId::ack;
        d += 1;
        return true;
    } else if (b0 == RailComSpec::DecId::dec_nak) {
        id = MsgId::nak;
        d += 1;
        return true;
#if RAILCOMSPEC_VERSION == 2012
    } else if (b0 == RailComSpec::DecId::dec_bsy) {
        id = MsgId::bsy;
        d += 1;
        return true;
#endif
    } else if (b0 < RailComSpec::DecId::dec_max) {
        // b0 was successfully decoded to 6 bits of data
        RailComSpec::PktId pkt_id = RailComSpec::PktId((b0 >> 2) & 0x0f);
        if (pkt_id == RailComSpec::PktId::pkt_pom) {
            // 12 bit (2 byte) message
            if (len >= 2) {
                uint8_t b1 = d[1];
                if (b1 < RailComSpec::DecId::dec_max) {
                    id = MsgId::pom;
                    pom.val = ((b0 << 6) | b1) & 0xff;
                    d += 2;
                    return true;
                }
            }
            // it looks like ahi and alo are allowed in either channel
        } else if (pkt_id == RailComSpec::PktId::pkt_ahi) {
            // 12 bit (2 byte) message
            if (len >= 2) {
                uint8_t b1 = d[1];
                if (b1 < RailComSpec::DecId::dec_max) {
                    id = MsgId::ahi;
                    ahi.ahi = ((b0 << 6) | b1) & 0xff;
                    d += 2;
                    return true;
                }
            }
        } else if (pkt_id == RailComSpec::PktId::pkt_alo) {
            // 12 bit (2 byte) message
            if (len >= 2) {
                uint8_t b1 = d[1];
                if (b1 < RailComSpec::DecId::dec_max) {
                    id = MsgId::alo;
                    alo.alo = ((b0 << 6) | b1) & 0xff;
                    d += 2;
                    return true;
                }
            }
        } else if (pkt_id == RailComSpec::PktId::pkt_ext) {
            // 18 bit (3 byte) message
            if (len >= 3) {
                uint8_t b1 = d[1];
                uint8_t b2 = d[2];
                if (b1 < RailComSpec::DecId::dec_max &&
                    b2 < RailComSpec::DecId::dec_max) {
                    id = MsgId::ext;
                    ext.typ = ((b0 << 4) & 0x30) | ((b1 >> 2) & 0x0f);
                    ext.pos = ((b1 << 6) & 0xc0) | b2;
                    d += 3;
                    return true;
                }
            }
        } else if (pkt_id == RailComSpec::PktId::pkt_dyn) {
            // 18 bit (3 byte) message
            if (len >= 3) {
                uint8_t b1 = d[1];
                uint8_t b2 = d[2];
                if (b1 < RailComSpec::DecId::dec_max &&
                    b2 < RailComSpec::DecId::dec_max) {
                    id = MsgId::dyn;
                    dyn.val = ((b0 << 6) | b1) & 0xff;
                    dyn.id = RailComSpec::DynId(b2);
                    d += 3;
                    return true;
                }
            }
        } else if ((pkt_id & 0x0c) == RailComSpec::PktId::pkt_xpom) {
            // xpom 8, 9, 10, 11 (0x08, 0x09, 0x0a, 0x0b)
            // 36 bit (6 byte) message
            if (len >= 6) {
                uint8_t b1 = d[1];
                uint8_t b2 = d[2];
                uint8_t b3 = d[3];
                uint8_t b4 = d[4];
                uint8_t b5 = d[5];
                if (b1 < RailComSpec::DecId::dec_max &&
                    b2 < RailComSpec::DecId::dec_max &&
                    b3 < RailComSpec::DecId::dec_max &&
                    b4 < RailComSpec::DecId::dec_max &&
                    b5 < RailComSpec::DecId::dec_max) {
                    // [ d0 ] [ d1 ] [ d2 ] [ d3 ] [ d4 ] [ d5 ]
                    // IIII00 000000 111111 112222 222233 333333
                    //     [ val0  ] [ val1  ][ val2  ][ val3  ]
                    id = MsgId::xpom;
                    xpom.ss = pkt_id & 0x03;
                    xpom.val[0] = (b0 << 6) | b1;
                    xpom.val[1] = (b2 << 2) | (b3 >> 4);
                    xpom.val[2] = (b3 << 4) | (b4 >> 2);
                    xpom.val[3] = (b4 << 6) | b5;
                    d += 6;
                    return true;
                }
            }
        }
    }
    return false;
}


// Pretty-print to buf
// Return number of characters written to buf
// Example output:
//   "[ACK]"
//   "[POM 1e]"
//   "[ALO 03]"
//   "[DYN SPD=0]"
int RailComMsg::show(char *buf, int buf_len) const
{
    memset(buf, '\0', buf_len);

    char *b = buf;
    char *e = buf + buf_len;

    b += snprintf(b, e - b, "[");

    b += snprintf(b, e - b, id_name());

    if (id == MsgId::ack || id == MsgId::nak || id == MsgId::bsy) {
        // nothing more to print
    } else if (id == MsgId::pom) {
        b += snprintf(b, e - b, " %02x", pom.val);
    } else if (id == MsgId::ahi) {
        b += snprintf(b, e - b, " %02x", ahi.ahi);
    } else if (id == MsgId::alo) {
        b += snprintf(b, e - b, " %02x", alo.alo);
    } else if (id == MsgId::ext) {
        b += snprintf(b, e - b, " %02x %02x", ext.typ, ext.pos);
    } else if (id == MsgId::dyn) {
        b += snprintf(b, e - b, " %s=%d", RailComSpec::dyn_name(dyn.id),
                      dyn.val);
    } else if (id == MsgId::xpom) {
        b += snprintf(b, e - b, " %d %02x %02x %02x %02x", xpom.ss, xpom.val[0],
                      xpom.val[1], xpom.val[2], xpom.val[3]);
    } else {
        b += snprintf(b, e - b, " ?");
    }

    b += snprintf(b, e - b, "]");

    return b - buf;
}


const char *RailComMsg::id_name() const
{
    static constexpr int id_max = int(MsgId::inv) + 1;
    static constexpr int name_max = 4;
    static char names[id_max][name_max] = {
        // ack nak bsy pom  ahi  alo  ext  dyn xpom  inv
        "A", "N", "B", "C", "H", "L", "E", "D", "X", "I"};
    xassert(int(id) < id_max);
    return names[int(id)];
}


bool RailComMsg::operator==(const RailComMsg &rhs) const
{
    if (id != rhs.id) {
        return false;
    }

    switch (id) {

        case MsgId::ack:
        case MsgId::nak:
        case MsgId::bsy:
            return true;

        case MsgId::pom:
            return pom.val == rhs.pom.val;

        case MsgId::ahi:
            return ahi.ahi == rhs.ahi.ahi;

        case MsgId::alo:
            return alo.alo == rhs.alo.alo;

        case MsgId::ext:
            return ext.typ == rhs.ext.typ && ext.pos == rhs.ext.pos;

        case MsgId::dyn:
            return dyn.id == rhs.dyn.id && dyn.val == rhs.dyn.val;

        case MsgId::xpom:
            return xpom.ss == rhs.xpom.ss && xpom.val[0] == rhs.xpom.val[0] &&
                   xpom.val[1] == rhs.xpom.val[1] &&
                   xpom.val[2] == rhs.xpom.val[2] &&
                   xpom.val[3] == rhs.xpom.val[3];

        default:
            return false;

    } // switch (id)

} // RailComMsg::operator==
