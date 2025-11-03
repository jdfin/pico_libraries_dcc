#pragma once

// Cmake adds -DPICO_BOARD="pico" or -DPICO_BOARD="pimoroni_tiny2040". That
// name (pico or pimoroni_tiny2040) is the filename of an include file (pico.h
// or pimoroni_tiny2040.h) in .pico-sdk/sdk/2.2.0/src/boards/include/boards.
// That file defines something like RASPBERRYPI_PICO or PIMORONI_TINY2040.
// Including pico.h ends up getting to the board file.
// (add_compile_options(-H) after pico_sdk_init() in top-level CMakeLists.txt)

#include "pico.h"

#if (defined RASPBERRYPI_PICO)

// drives DCC

// breadboard - encodes/drives DCC, reads/decodes RailCom
static const int dcc_sig_gpio = 19; // PH
static const int dcc_pwr_gpio = 18; // EN
static const int dcc_slp_gpio = -1; // SLP
static const int dcc_adc_gpio = 26; // CS (ADC0)

#elif (defined PIMORONI_TINY2040)

// reads/decodes DCC

static const int dcc_sig_gpio = 7;
static const int dcc_pwr_gpio = -1;
static const int dcc_slp_gpio = -1;
static const int dcc_adc_gpio = -1;

#else

#error Unknown board!

#endif
