#pragma once

#include <cstdint>
#include <cstring>
#include "hardware/uart.h"


class DccRailCom
{
public:
    DccRailCom(uart_inst_t* uart, int rx_gpio);

    void reset()
    {
        if (_uart == nullptr || _rx_gpio < 0)
            return;
        uart_deinit(_uart);
        uart_init(_uart, baud);
    }

    void read();

    void channelize();

    bool got_pkt() const
    {
        return _pkt_len == pkt_max;
    }

    char *dump(char *buf, int buf_len) const; // raw

    char *show(char *buf, int buf_len) const; // cooked

private:
    uart_inst_t* _uart;
    int _rx_gpio;

    static constexpr uint baud = 250'000;

    // data received in one cutout

    static constexpr int pkt_max = 8; // for both enc and dec
    uint8_t _enc[pkt_max]; // encoded (4/8 code)
    uint8_t _dec[pkt_max]; // decoded (6 bits per byte)
    int _pkt_len; // both _enc and _dec
    bool _pkt_valid; // true if there are no invalid bytes in _enc

    void pkt_reset()
    {
        memset(_enc, 0, sizeof(_enc));
        memset(_dec, 0, sizeof(_dec));
        _pkt_len = 0;
        _pkt_valid = true; // set false when an invalid byte is seen
    }

    // channelized

    uint _ch1_id;
    uint _ch1_data;
    bool _ch1_valid;

    void ch1_reset()
    {
        _ch1_id = 0;
        _ch1_data = 0;
        _ch1_valid = false;
    }

    static constexpr int ch2_len = 6;
    uint8_t _ch2[ch2_len];
    bool _ch2_valid;

    void ch2_reset()
    {
        memset(_ch2, 0, sizeof(_ch2));
        _ch2_valid = false;
    }

    static const char *dyn_name(uint x);

    // Lookup table: maps 4/8 code (as index) to decoded value
    // Usage: uint8_t value = decode[uint8_t four_eight_code];
    // invalid codes return 0xff.
    static constexpr uint8_t nack = 0x40;   // never seen from LokSound 5
    static constexpr uint8_t ack = 0x41;
    static constexpr uint8_t busy = 0x42;   // never seen from LokSound 5
    static constexpr uint8_t resv = 0x43;   // reserved
    static constexpr uint8_t inv = 0xff;
    static const uint8_t decode[UINT8_MAX + 1];

    // RailCom packet IDs (4 bits).
    // These are just the ones seen from LokSound 5.
    static constexpr uint id_pom = 0;
    static constexpr uint id_ahi = 1;
    static constexpr uint id_alo = 2;
    static constexpr uint id_dyn = 7;
};