#pragma once

#include <cstdint>
#include <cstring>

#include "hardware/uart.h"
#include "railcom_msg.h"
#include "railcom_spec.h"


class RailCom
{

public:

    RailCom(uart_inst_t *uart, int rx_gpio);

    void reset() // called in interrupt context
    {
        if (_uart == nullptr || _rx_gpio < 0) {
            return;
        }
        uart_deinit(_uart);
        uart_init(_uart, RailComSpec::baud);
    }

    void read();

    void parse();

    char *dump(char *buf, int buf_len) const; // raw

    char *show(char *buf, int buf_len) const; // pretty

    int get_ch2_msgs(const RailComMsg *&msgs) {
        msgs = _ch2_msg;
        return _ch2_msg_cnt;
    }

private:

    uart_inst_t *_uart;

    int _rx_gpio;

    ///// Raw RailCom Data (4/8 encoded, and decoded bytes)

    static constexpr int pkt_max = RailComSpec::ch1_bytes + RailComSpec::ch2_bytes;
    uint8_t _enc[pkt_max]; // encoded (4/8 code)
    uint8_t _dec[pkt_max]; // decoded (6 bits per byte) from decode[]
    int _pkt_len;          // _enc[] and _dec[] are the same length

    ///// Parsed RailCom Messages

    // channel 1 messages
    RailComMsg _ch1_msg;
    int _ch1_msg_cnt; // 0 or 1

    // channel 2 messages
    // at most one message per byte (e.g. all ACK)
    static constexpr int ch2_msg_max = RailComSpec::ch2_bytes;
    RailComMsg _ch2_msg[ch2_msg_max];
    int _ch2_msg_cnt; // 0...ch2_msg_max-1

    // true if there's no junk left over after parsing
    bool _parsed_all;

    ///// Debug

    // These are used to assert a GPIO on some event to trigger a scope.
    // All default to -1 (disabled).
    static int dbg_read;    // asserted for duration of RailCom::read()
    static int dbg_junk;    // asserted when invalid bytes received
    static int dbg_short;   // asserted when less than pkt_max bytes received
    static void dbg_init(); // call this after changing any from default

}; // class RailCom
