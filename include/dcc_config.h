#pragma once

// Cmake adds -DPICO_BOARD="pico" or -DPICO_BOARD="pimoroni_tiny2040". That
// name (pico or pimoroni_tiny2040) is the filename of an include file (pico.h
// or pimoroni_tiny2040.h) in .pico-sdk/sdk/2.2.0/src/boards/include/boards.
// That file defines something like RASPBERRYPI_PICO or PIMORONI_TINY2040.
// Including pico.h ends up getting to the board file.
// (add_compile_options(-H) after pico_sdk_init() in top-level CMakeLists.txt)

#include "pico.h"
#include "hardware/uart.h"

#if (defined RASPBERRYPI_PICO)

// drives DCC

// breadboard - encodes/drives DCC, reads/decodes RailCom
constexpr int dcc_sig_gpio = 19; // PH
constexpr int dcc_pwr_gpio = 18; // EN
constexpr int dcc_slp_gpio = -1; // SLP
constexpr int dcc_adc_gpio = 26; // CS (ADC0)

constexpr int railcom_gpio = 17;
uart_inst_t * const railcom_uart = uart0;

// position 14 is gpio 20
// position 15 is gpio 21
constexpr int railcom_dbg_read_gpio = -1;
constexpr int railcom_dbg_junk_gpio = -1;
constexpr int railcom_dbg_short_gpio = -1;
constexpr int dcc_bitstream_dbg_next_bit_gpio = 21;
constexpr int dcc_command_dbg_get_packet_gpio = 20;

#elif (defined PIMORONI_TINY2040)

// reads/decodes DCC

static constexpr int dcc_sig_gpio = 7;
static constexpr int dcc_pwr_gpio = -1;
static constexpr int dcc_slp_gpio = -1;
static constexpr int dcc_adc_gpio = -1;

constexpr int railcom_gpio = 17;
uart_inst_t * const railcom_uart = uart0;

static constexpr int railcom_dbg_read_gpio = -1;
static constexpr int railcom_dbg_junk_gpio = -1;
static constexpr int railcom_dbg_short_gpio = -1;
static constexpr int dcc_bitstream_dbg_next_gpio = -1;

#else

#error Unknown board!

#endif
