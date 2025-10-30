
#include "dcc_railcom.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "xassert.h"

#include "dbg_gpio.h"

static const int dbg_gpio = 21;

//#define RAILCOM_VERSION 2012
#define RAILCOM_VERSION 2021

#if (RAILCOM_VERSION != 2012) && (RAILCOM_VERSION != 2021)
#error RAILCOM_VERSION UNKNOWN
#endif

const uint8_t DccRailCom::decode[UINT8_MAX + 1] = {
    inv, inv, inv, inv, inv, inv, inv, inv, // 0x00-0x07
    inv, inv, inv, inv, inv, inv, inv, // 0x08-0x0e
#if RAILCOM_VERSION == 2012
    nack, // 0x0f
#elif RAILCOM_VERSION == 2021
    ack, // 0x0f (there are two ACKs)
#endif
    inv, inv, inv, inv, inv, inv, inv, 0x33, // 0x10-0x17
    inv, inv, inv, 0x34, inv, 0x35, 0x36, inv, // 0x18-0x1f
    inv, inv, inv, inv, inv, inv, inv, 0x3a, // 0x20-0x27
    inv, inv, inv, 0x3b, inv, 0x3c, 0x37, inv, // 0x28-0x2f
    inv, inv, inv, 0x3f, inv, 0x3d, 0x38, inv, // 0x30-0x37
    inv, 0x3e, 0x39, inv, // 0x38-0x3b
#if RAILCOM_VERSION == 2012
    resv, // 0x3c
#elif RAILCOM_VERSION == 2021
    nack, // 0x3c (optional)
#endif
    inv, inv, inv,   // 0x3d-0x3f
    inv, inv, inv, inv, inv, inv, inv, 0x24,  // 0x40-0x47
    inv, inv, inv, 0x23, inv, 0x22, 0x21, inv,   // 0x48-0x4f
    inv, inv, inv, 0x1f, inv, 0x1e, 0x20, inv,   // 0x50-0x57
    inv, 0x1d, 0x1c, inv, 0x1b, inv, inv, inv,   // 0x58-0x5f
    inv, inv, inv, 0x19, inv, 0x18, 0x1a, inv,   // 0x60-0x67
    inv, 0x17, 0x16, inv, 0x15, inv, inv, inv,   // 0x68-0x6f
    inv, 0x25, 0x14, inv, 0x13, inv, inv, inv,   // 0x70-0x77
    0x32, inv, inv, inv, inv, inv, inv, inv,   // 0x78-0x7f
    inv, inv, inv, inv, inv, inv, inv, resv, // 0x80-0x87
    inv, inv, inv, 0x0e, inv, 0x0d, 0x0c, inv,   // 0x88-0x8f
    inv, inv, inv, 0x0a, inv, 0x09, 0x0b, inv,   // 0x90-0x97
    inv, 0x08, 0x07, inv, 0x06, inv, inv, inv,   // 0x98-0x9f
    inv, inv, inv, 0x04, inv, 0x03, 0x05, inv,   // 0xa0-0xa7
    inv, 0x02, 0x01, inv, 0x00, inv, inv, inv,   // 0xa8-0xaf
    inv, 0x0f, 0x10, inv, 0x11, inv, inv, inv,   // 0xb0-0xb7
    0x12, inv, inv, inv, inv, inv, inv, inv,   // 0xb8-0xbf
    inv, inv, inv, resv, inv, 0x2b, 0x30, inv,   // 0xc0-0xc7
    inv, 0x2a, 0x2f, inv, 0x31, inv, inv, inv,   // 0xc8-0xcf
    inv, 0x29, 0x2e, inv, 0x2d, inv, inv, inv,   // 0xd0-0xd7
    0x2c, inv, inv, inv, inv, inv, inv, inv,   // 0xd8-0xdf
    inv, // 0xe0
#if RAILCOM_VERSION == 2012
    busy, // 0xe1
#elif RAILCOM_VERSION == 2021
    resv, // 0xe1
#endif
    0x28, inv, 0x27, inv, inv, inv,   // 0xe2-0xe7
    0x26, inv, inv, inv, inv, inv, inv, inv,   // 0xe8-0xef
    ack, inv, inv, inv, inv, inv, inv, inv,   // 0xf0-0xf7
    inv, inv, inv, inv, inv, inv, inv, inv,   // 0xf8-0xff
};


DccRailCom::DccRailCom(uart_inst_t* uart, int rx_gpio) :
    _uart(uart),
    _rx_gpio(rx_gpio)
{
    if (_uart == nullptr || _rx_gpio < 0)
        return;

    if (dbg_gpio >= 0)
        DbgGpio::init(dbg_gpio);

    gpio_set_function(_rx_gpio, UART_FUNCSEL_NUM(_uart, _rx_gpio));
    uart_init(_uart, baud);

    pkt_reset();
    ch1_reset();
    ch2_reset();
}

void DccRailCom::read()
{
    pkt_reset();
    ch1_reset();
    ch2_reset();

    for (_pkt_len = 0; _pkt_len < pkt_max && uart_is_readable(_uart); _pkt_len++) {
        _enc[_pkt_len] = uart_getc(_uart);
        _dec[_pkt_len] = decode[_enc[_pkt_len]];
        if (_dec[_pkt_len] == inv) {
            _pkt_valid = false;
            if (dbg_gpio >= 0) {
                DbgGpio d(dbg_gpio); // scope trigger
                // this seems to be needed to force construction of DbgGpio
                [[maybe_unused]] volatile int i = 0;
            }
        }
    }

    if (_pkt_len != pkt_max) {
        DbgGpio d(dbg_gpio);
        [[maybe_unused]] volatile int i = 0;
    }
}

// Split received packet into channel 1 and channel 2
//
// Channel 1 is by default always sent by all decoders that support RailCom,
// but that can be disabled in the decoder. If there is more than one loco on
// the same track, they will both send channel 1 and it will likely be junk.
// We don't use it, but decoding it helps figure out where channel 2 starts.
//
// Channel 2 is only sent by the DCC-addressed decoder. If there is no decoder
// at the DCC address of DCC packet, there will be no channel 2 data. If there
// is an addressed decoder, it will send channel 2 data, but it is often
// corrupted, presumably by dirty track and such. Observed corruption seems to
// extra ones in the 4/8 encoding, implying the decoder was trying to send a
// zero (>10 mA), but it did not get through (e.g. because of dirty track).
// Multiple decoders at the same DCC address would also cause corruption, but
// with excess zeros instead of excess ones. Channel 2 often does not need
// the full 6 bytes. It would be possible to use information from channel 2
// with only one byte, e.g. an ack, but a choice here is to require 6 valid
// bytes to consider anything in channel 2 valid.
//
// If we got at least 2 bytes, see if the first two are valid channel 1 data.
//
// If we got a valid channel 1,
//   try extracting 6 bytes of channel 2 data starting at byte 2,
// else (channel 1 invalid),
//   try extracting 6 bytes of channel 2 data starting at byte 0.
//
// If channel 1 failed due to corruption, channel 2 will also fail due to
// corruption. If channel 1 failed because no decoder sent it (i.e. it is
// disabled), we probably got 6 bytes and it's all channel 2.
//
// XXX Can we get less than 6 bytes of channel 2 data? ESU LokSound 5 fills
//     out channel 2 to 6 bytes, but I don't think the spec requires that.
//
void DccRailCom::channelize()
{
    int ch2_start = 0;

    // don't care about _pkt_valid; might still be able to extract something

    // extract channel 1
    ch1_reset();
    if (_pkt_len >= 2 && _dec[0] < 0x40 && _dec[1] < 0x40) {
        uint id = (_dec[0] >> 2) & 0x0f;
        if (id == id_ahi || id == id_alo) {
            _ch1_id = id;
            _ch1_data = ((_dec[0] << 6) | _dec[1]) & 0xff;
            _ch1_valid = true;
            ch2_start = 2;
        }
    }

    // extract channel 2
    _ch2_valid = true;
    if ((ch2_start + ch2_len) != _pkt_len) {
        ch2_reset();
    } else {
        for (int i = 0; i < ch2_len; i++) {
            if (_dec[ch2_start + i] != inv) {
                _ch2[i] = _dec[ch2_start + i];
            } else {
                ch2_reset();
                break;
            }
        }
    }
}

// for each byte:
//   if a byte is not 4/8 valid
//     show raw !hh!
//   else
//     decode to 6 bits
//     if >= 0x40
//       show raw [hh]
//     else
//       show decoded bbbbbb
char* DccRailCom::dump(char* buf, int buf_len) const
{
    memset(buf, '\0', buf_len);

    char* b = buf;
    char* e = buf + buf_len;

    for (int i = 0; i < _pkt_len; i++) {
        if (i > 0)
            b += snprintf(b, e - b, " ");
        uint8_t d = decode[_enc[i]];
        if (d == inv) {
            b += snprintf(b, e - b, "%02x", _enc[i]);
        } else if (d >= 0x40) {
            b += snprintf(b, e - b, "%02x", _enc[i]);
        } else {
            for (uint8_t m = 0x20; m != 0; m >>= 1) {
                b += snprintf(b, e - b, "%c", (d & m) != 0 ? '1' : '0');
            }
        }
    }

    return buf;
}

char* DccRailCom::show(char* buf, int buf_len) const
{
    memset(buf, '\0', buf_len);

    char* b = buf;
    char* e = buf + buf_len;

    b += snprintf(b, e - b, "R ");

    // it is expected that channelize() has been called before this
    // XXX enforce

    // show channel 1
    if (_ch1_valid) {
        if (_ch1_id == id_ahi) {
            b += snprintf(b, e - b, "AHI");
        } else if (_ch1_id == id_alo) {
            b += snprintf(b, e - b, "ALO");
        } else {
            b += snprintf(b, e - b, "0x%x", _ch1_id);
        }
        b += snprintf(b, e - b, "=0x%02x | ", _ch1_data);
    } else {
        b += snprintf(b, e - b, "/ch1/    | ");
    }

    // show channel 2
    if (_ch2_valid) {
        if (_ch2[0] >= 0x40) {
            if (_ch2[0] == ack) {
                b += snprintf(b, e - b, "ACK ");
            } else if (_ch2[0] == nack) {
                b += snprintf(b, e - b, "NAK ");
            } else if (_ch2[0] == busy) {
                b += snprintf(b, e - b, "BSY ");
            } else {
                b += snprintf(b, e - b, "%02x ", uint(_ch2[0]));
            }
        } else {
            // first 4 bits are always ID
            uint id = (_ch2[0] >> 2) & 0x0f;
            if (id == id_pom) {
                // POM
                uint cv_val = ((_ch2[0] << 6) | _ch2[1]) & 0xff;
                // Information is in _ch2[0..1]; _ch2[2..5] is unused.
                // It is possible to write two CVs at once and get two of these in one response, but we're not there yet.
                // Print ID and value.
                b += snprintf(b, e - b, "POM %02x ", cv_val);
            } else if (id == id_dyn) {
                // DYN
                // After ID comes 8 bits of data, then 6 bits of index, then maybe another ID/data/index.
                // IDID DATADATA INDEXX IDID DATADATA INDEXX
                // [  0  ][  1 ] [  2 ] [  3  ][  4 ] [  5 ]
                uint val = ((_ch2[0] << 6) | _ch2[1]) & 0xff;
                uint idx = _ch2[2];
                b += snprintf(b, e - b, "DYN %s=%u ", dyn_name(idx), val);
                // another one?
                id = (_ch2[3] >> 2) & 0x0f;
                if (id == id_dyn) {
                    val = ((_ch2[3] << 6) | _ch2[4]) & 0xff;
                    idx = _ch2[5];
                    b += snprintf(b, e - b, "DYN %s=%u ", dyn_name(idx), val);
                }
            } else {
                // unexpected ID
                b += snprintf(b, e - b, "%u: ?? ", id);
            }
        }
    } else {
        b += snprintf(b, e - b, "/ch2/ ");
    }

    if (!_pkt_valid || !_ch1_valid || !_ch2_valid) {
        // print raw 4/8 encoded data and return
        b += snprintf(b, e - b, "[ ");
        if (_pkt_len > 0) {
            for (int i = 0; i < _pkt_len; i++)
                b += snprintf(b, e - b, "%02x ", uint(_enc[i]));
        } else {
            b += snprintf(b, e - b, "no data ");
        }
        b += snprintf(b, e - b, "] ");
    }

    return buf;

} // DccRailCom::show()

const char *DccRailCom::dyn_name(uint x)
{
    static constexpr uint x_max = 64;
    static constexpr uint name_max = 8;
    static char id_names[x_max][name_max] = {
        "SPD1", "SPD2", "ID_2", "ID_3", "ID_4", "ID_5", "ID_6", "ID_7",
        "ID_8", "ID_9", "ID10", "ID11", "ID12", "ID13", "ID14", "ID15",
        "ID16", "ID17", "ID18", "ID19", "ID20", "ID21", "ID22", "ID23",
        "ID24", "ID25", "ID26", "ID27", "ID28", "ID29", "ID30", "ID31",
        "ID32", "ID33", "ID34", "ID35", "ID36", "ID37", "ID38", "ID39",
        "ID40", "ID41", "ID42", "ID43", "ID44", "ID45", "ID46", "ID47",
        "ID48", "ID49", "ID50", "ID51", "ID52", "ID53", "ID54", "ID55",
        "ID56", "ID57", "ID58", "ID59", "ID60", "ID61", "ID62", "ID63",
    };

    xassert(x < x_max);
    return id_names[x];
}