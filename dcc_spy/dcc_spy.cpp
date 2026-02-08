
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "dcc_bit.h"
#include "dcc_gpio_cfg.h"
#include "dcc_pkt.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pio_edges.h"
#include "sys_led.h"

#undef INCLUDE_DISPLAY

#ifdef INCLUDE_DISPLAY
#include "consolas_24.h"
#include "consolas_italic_24.h"
#include "ws24.h"
#endif

static const char *prog_name = "DCC Spy";

static const int verbosity = 0;

// PIO timestamp resolution (will ask Edges for it)
static uint32_t pio_tick_hz = 0;
static uint32_t pio_tick_ns = 0;

// Rising edges are adjusted for the slow rise time (hardware thing)
static const uint32_t adj_ns = 440;
static uint32_t adj_tk = 0;

static DccBit dcc(verbosity);

static void pkt_recv(const uint8_t *pkt, int pkt_len, int preamble_len,
                     uint64_t start_us, int bad_cnt);

#ifdef INCLUDE_DISPLAY

// display gpio pins
static const int gpio_spi_miso = 0;
static const int gpio_spi_cs = 1;
static const int gpio_spi_clk = 2;
static const int gpio_spi_mosi = 3;
static const int gpio_lcd_dc = 4;
static const int gpio_lcd_bl = 5;
static const int gpio_lcd_reset = 6;

static const int spi_clk_hz = 10'000'000;

// Data structure with data to display.
// Written by core0 as packets arrive, read by core1 at its leisure.
struct DisplayData {
    DisplayData()
    {
        reset();
    }
    void reset()
    {
        address = DccPkt::address_inv;
        speed = DccPkt::speed_inv;
        for (int i = 0; i < f_max; i++) {
            f[i] = -1;
        }
    }
    int address;
    uint32_t address_ms; // last time we saw any message to this address
    int speed;
    uint32_t speed_ms; // last time we saw speed message to this address
    static const int f_max = 29;
    int f[f_max];
    uint32_t f0_ms;  // f0..f4
    uint32_t f5_ms;  // f5..f8
    uint32_t f9_ms;  // f9..f12
    uint32_t f13_ms; // f13..f20
    uint32_t f21_ms; // f21..f28
};

volatile DisplayData mc_shared;

static void core1_main();

#endif // INCLUDE_DISPLAY


static void init()
{
    stdio_init_all();

    SysLed::init();
    SysLed::pattern(50, 950);

    while (!stdio_usb_connected()) {
        tight_loop_contents();
        SysLed::loop();
    }

    // With no delay here, we lose the first few lines of output.
    // Delaying 1 msec has been observed to work with a debug build.
    sleep_ms(10);

    SysLed::pattern(50, 1950);

    printf("\n");
    printf("%s on GPIO %d\n", prog_name, dcc_sig_gpio);
    printf("\n");

    Edges::init(dcc_sig_gpio);

    pio_tick_hz = Edges::get_tick_hz();
    assert(pio_tick_hz > 0);

    pio_tick_ns = 1'000'000'000 / pio_tick_hz;
    assert(pio_tick_ns > 0);

    //printf("pio_tick_hz = %u, pio_tick_ns = %u\n", pio_tick_hz, pio_tick_ns);
    assert((pio_tick_hz * pio_tick_ns) == 1'000'000'000);

    adj_tk = (adj_ns + pio_tick_ns / 2) / pio_tick_ns;

#ifdef INCLUDE_DISPLAY
    multicore_launch_core1(core1_main);
#endif

    dcc.on_pkt_recv(&pkt_recv);

    dcc.init();
}


static void loop()
{
    int rise;
    uint64_t edge64_tk;
    if (!Edges::get_tick(rise, edge64_tk)) {
        return;
    }

    // Adjust rising edges for slow rise time (hardware thing)
    if (rise == 1) {
        edge64_tk -= adj_tk;
    }

    // convert from ticks to microseconds (with rounding)
    uint64_t edge64_us = (edge64_tk * pio_tick_ns + 500) / 1000;

    // NOTE: edge64_us and the return from time_us_64() are offset from each other
    // They should tick at the same rate but should not be compared.

    // dcc doesn't care if it's a rising or falling edge
    dcc.edge(edge64_us);
}


#ifdef INCLUDE_DISPLAY


// This is core0 writing the shared data structure that core1 will use to
// update the display
static void update_display(const DccPkt &msg)
{
    int new_address = msg.get_address();

    // Ignore broadcast (0) and "invalid" (INT_MAX).
    // This really should be ignoring much more, anything that is not multi-
    // function decoder, either 7- or 14-bit. Or, lots of stuff should be
    // updated to handle other types of address.
    if (new_address < DccPkt::address_min ||
        new_address > DccPkt::address_max) {
        return;
    }

    // start_us is from the pio tick and is different from time_us_64()
    uint32_t now_ms = (time_us_64() + 500) / 1000; // ok if now_ms wraps

    mc_shared.address = new_address;
    mc_shared.address_ms = now_ms;

    int speed;
    int f[8]; // decode functions use 4, 5, or 8 entries

    if (msg.decode_speed_128(speed)) {
        mc_shared.speed = speed;
        mc_shared.speed_ms = now_ms;
    } else if (msg.decode_func_0(f)) {
        // f0..f4 (5 entries)
        for (int i = 0; i < 5; i++) {
            mc_shared.f[i] = f[i];
        }
        mc_shared.f0_ms = now_ms;
    } else if (msg.decode_func_5(f)) {
        // f5..f8 (4 entries)
        for (int i = 0; i < 4; i++) {
            mc_shared.f[5 + i] = f[i];
        }
        mc_shared.f5_ms = now_ms;
    } else if (msg.decode_func_9(f)) {
        // f9..f13 (4 entries)
        for (int i = 0; i < 4; i++) {
            mc_shared.f[9 + i] = f[i];
        }
        mc_shared.f9_ms = now_ms;
    } else if (msg.decode_func_13(f)) {
        // f13..20 (8 entries)
        for (int i = 0; i < 8; i++) {
            mc_shared.f[13 + i] = f[i];
        }
        mc_shared.f13_ms = now_ms;
    } else if (msg.decode_func_21(f)) {
        // f21..28 (8 entries)
        for (int i = 0; i < 8; i++) {
            mc_shared.f[21 + i] = f[i];
        }
        mc_shared.f21_ms = now_ms;
    }

} // static void update_display()


#endif // INCLUDE_DISPLAY

static bool pkt_ignore(const uint8_t *pkt, int pkt_len)
{
    // ignore packets that are not multi-function decoder speed or function messages
    DccPkt msg(pkt, pkt_len);

    int speed;
    int f[8];
    return msg.decode_speed_128(speed) // ignore any of these
           || msg.decode_func_0(f)     //
           || msg.decode_func_5(f)     //
           || msg.decode_func_9(f)     //
           || msg.decode_func_13(f)    //
           || msg.decode_func_21(f)    //
           || msg.decode_func_29(f);
}


static void pkt_recv(const uint8_t *pkt, int pkt_len, int preamble_len,
                     uint64_t start_us, int bad_cnt)
{
    static uint64_t last_pkt_us = 0;

    uint8_t check = 0;
    for (int i = 0; i < pkt_len; i++)
        check ^= pkt[i];

    if (!pkt_ignore(pkt, pkt_len)) {

        printf("%8llu %8llu p: %d pkt:", start_us, start_us - last_pkt_us,
               preamble_len);

        for (int i = 0; i < pkt_len; i++)
            printf(" %02x", pkt[i]);

        for (int i = pkt_len; i < 6; i++)
            printf("   ");

        printf(" (%s)", (check == 0) ? "ok" : "error");

        DccPkt msg(pkt, pkt_len);

        char buf[80];
        printf(" %s", msg.show(buf, sizeof(buf)));

        if (bad_cnt != 0)
            printf(" bad_cnt=%d", bad_cnt);

        printf("\n");

#ifdef INCLUDE_DISPLAY
        update_display(msg);
#endif
    }

    last_pkt_us = start_us;

} // static void pkt_recv(...)


#ifdef INCLUDE_DISPLAY


[[maybe_unused]]
static void lcd_clear(Ws24 &lcd, const Font &font)
{
    // erase all but the top row where the title is
    lcd.write(font.height(), 0, lcd.height() - font.height(), lcd.width(),
              Pixel::white);
}


// f0..f9 on row 2, f10..f19 on row 3, f20..f28 on row 4
static inline int f_row(const Font &font, int f)
{
    return (2 + f / 10) * font.height();
}


static int f_col(Ws24 &lcd, const Font &font, int f)
{
    static const int cell_w = lcd.width() / 10;

    if (f < 10) {
        return (f % 10) * cell_w + cell_w / 2 -
               font.max_width() / 2; // 1 char wide
    } else {
        return (f % 10) * cell_w + cell_w / 2 -
               font.max_width(); // 2 chars wide
    }
}


// core1 looks for changes in status and updates the screen
static void core1_main()
{
    const Font &font = consolas_24;
    const Font &font_it = consolas_italic_24;
    DisplayData current;
    char buf[32];

    bool speed_stale = false;

    static_assert(consolas_24_max_height == consolas_italic_24_max_height &&
                      consolas_24_max_width == consolas_italic_24_max_width,
                  "normal and italic fonts must have the same dimensions");

    // for F0..F28
    const Pixel active =
        Pixel::shade(Pixel::red, Pixel::black, 25); // darken red just a tad
    const Pixel inactive =
        Pixel::shade(Pixel::white, Pixel::black, 10); // barely there

    const int row_height = font.height();

    constexpr int work_bytes =
        consolas_24_max_height * consolas_24_max_width * sizeof(Pixel);
    uint8_t work[work_bytes];

    Ws24 lcd(gpio_spi_clk, gpio_spi_miso, gpio_spi_mosi, spi_clk_hz,
             gpio_spi_cs, gpio_lcd_dc, gpio_lcd_reset, gpio_lcd_bl, work,
             work_bytes);

    // rotate 90 left, backlight 100%
    lcd.begin(-90, 255);

    // clear
    lcd.write(0, 0, lcd.height(), lcd.width(), Pixel::white);

    uint16_t col = (lcd.width() - consolas_24.width(prog_name)) / 2;
    lcd.print(consolas_24, 0, col, Pixel::black, Pixel::white, prog_name);

    while (true) {

        uint32_t now_ms = (time_us_64() + 500) / 1000;

        if (current.address != mc_shared.address) {
            current.reset();
            current.address = mc_shared.address;
            //lcd_clear(lcd, font);
            sprintf(buf, "Loco %-5d", current.address);
            lcd.print(font, row_height, font.max_width() / 2, Pixel::black,
                      Pixel::white, buf);
        }

        if (current.speed != mc_shared.speed) {
            current.speed = mc_shared.speed; // -127..+128
            // all messages are at least 7 characters
            // leading spaces are to make sure the entire (non-space) previous message is overwritten
            if (current.speed == 0) {
                sprintf(buf, "   Stop");
            } else if (0 < current.speed &&
                       current.speed <= DccPkt::speed_max) {
                sprintf(buf, "  Fwd %d", current.speed);
            } else if (DccPkt::speed_min <= current.speed &&
                       current.speed < 0) {
                sprintf(buf, "  Rev %d", -current.speed);
            } else {
                sprintf(buf, "         ");
            }
            printf("speed: \"%s\"\n", buf);
            // right justified, max_width/2 from right edge
            lcd.print(font, row_height,
                      lcd.width() - font.max_width() / 2 - font.width(buf),
                      Pixel::black, Pixel::white, buf);
            current.speed_ms = now_ms;
            speed_stale = false;
        } else if (!speed_stale) {
            // speed has not changed - has it become stale?
            uint32_t age_ms = now_ms - mc_shared.speed_ms;
            if (age_ms >= 5000) {
                // speed is now stale
                if (current.speed == 0) {
                    sprintf(buf, "   Stop");
                } else if (0 < current.speed &&
                           current.speed <= DccPkt::speed_max) {
                    sprintf(buf, "  Fwd %d", current.speed);
                } else if (DccPkt::speed_min <= current.speed &&
                           current.speed < 0) {
                    sprintf(buf, "  Rev %d", -current.speed);
                } else {
                    sprintf(buf, "         ");
                }
                printf("stale: \"%s\"\n", buf);
                lcd.print(font_it, row_height,
                          lcd.width() - font.max_width() / 2 - font.width(buf),
                          inactive, Pixel::white, buf);
                speed_stale = true;
            }
        }

        for (int i = 0; i < DisplayData::f_max; i++) {
            if (current.f[i] != mc_shared.f[i]) {
                current.f[i] = mc_shared.f[i];
                sprintf(buf, "%d", i);
                lcd.print(font, f_row(font, i), f_col(lcd, font, i),
                          current.f[i] ? active : inactive, Pixel::white, buf);
            } else {
                // function has not changed - is it stale?
            }
        }

    } // while (true)

} // core1_main


#endif // INCLUDE_DISPLAY


int main()
{
    init();

    while (true) {
        SysLed::loop();
        loop();
    }

    return 0;
}
