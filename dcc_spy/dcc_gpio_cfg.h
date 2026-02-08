#pragma once

// Tiny2040
//
//                     +-----| USB |-----+
//                VBUS | 1            40 | D0
//                 GND | 2            39 | D1  CS     (fb)
//                 3V3 | 3            38 | D2  SCK    (fb)
//                 D29 | 4            37 | D3  MOSI   (fb)
//                 D28 | 5            36 | D4  DC     (fb)
//                 D27 | 6            35 | D5  BL     (fb)
//                 D26 | 7            34 | D6  RES    (fb)
//                 GND | 8            33 | D7  Signal (dcc)
//                     +-----------------+

constexpr int dcc_sig_gpio = 7;
