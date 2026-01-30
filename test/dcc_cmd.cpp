#include <strings.h>

#include <cassert>
#include <cstdint>
#include <cstdio>
// pico
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
// misc
#include "argv.h"
#include "str_ops.h"
#include "sys_led.h"
// dcc
#include "dcc_adc.h"
#include "dcc_bitstream.h"
#include "dcc_command.h"
#include "dcc_cv.h"
#include "dcc_gpio_cfg.h"
#include "dcc_pkt.h"
#include "dcc_throttle.h"
#include "railcom.h"

// Some commands, mainly service-mode reads and writes, take a while (a few
// hundred msec) to complete. When one of these is started, a function pointer
// is set to poll for progress/completion. Each time through the main loop()
// function, the "active" function is called; it is most often the no-op
// function, but is set to some other progress function when needed.

typedef bool(loop_func)();

static bool loop_nop();
static bool loop_ops_cv_read();
static bool loop_svc_cv_read();
static bool loop_svc_cv_write();
static bool loop_svc_address_read();
static bool loop_svc_address_write();

static loop_func *active = &loop_nop;

// A command is a sequence of tokens, ending with newline.
//
// The "Argv" object takes a character at a time and builds a complete command
// (an array of tokens), which can then be examined for commands and parameters.

static Argv argv;

// These functions look at commands and see if there are any valid commands
// to process.

static bool cmd_try();
static bool loco_try();
static bool speed_try();
static bool function_try();
static bool track_try();
static bool cv_try();
static bool verbosity_try();
static bool address_try();
static bool debug_try();

static void cmd_help(bool verbose = false);
static void loco_help(bool verbose = false);
static void speed_help(bool verbose = false);
static void function_help(bool verbose = false);
static void track_help(bool verbose = false);
static void cv_help(bool verbose = false);
static void verbosity_help(bool verbose = false);
static void address_help(bool verbose = false);
static void debug_help(bool verbose = false);
static void param_help();
static void print_help(bool verbose, const char *help_short,
                       const char *help_long);

// Verbosity
static bool cmd_show = true;

// DCC interface

static DccAdc adc(dcc_adc_gpio);
static DccCommand command(dcc_sig_gpio, dcc_pwr_gpio, -1, adc, dcc_rcom_uart,
                          dcc_rcom_gpio);
static DccThrottle *throttle = nullptr;

// When reading/writing CVs, the cv_num_g is set in one command and the read or
// write command is in the next. Global statics are used to save them.

static int cv_num_g = DccPkt::cv_num_inv;
static int cv_bit_g = -1;
static int cv_val_g = DccPkt::cv_val_inv;

static int address_g = DccPkt::address_inv;

// Throttle that issued the last ops mode cv read/write command
static DccThrottle *ops_throttle_g = nullptr;

// The start time of a long operation (read or write in service mode) is saved
// so the overall time can be printed.

static uint64_t start_us = 0;

int RailCom::dbg_read = dcc_dbg_rcom_read_gpio;
int RailCom::dbg_junk = dcc_dbg_rcom_junk_gpio;
int RailCom::dbg_short = dcc_dbg_rcom_short_gpio;
int DccBitstream::dbg_next_bit = dcc_dbg_bitstream_next_bit_gpio;
int DccCommand::dbg_get_packet = dcc_dbg_command_get_packet_gpio;


static inline uint32_t usec_to_msec(uint64_t us)
{
    return (uint32_t)((us + 500) / 1000);
}


int main()
{
    stdio_init_all();

    SysLed::init();
    SysLed::pattern(50, 950);

    while (!stdio_usb_connected()) {
        tight_loop_contents();
        SysLed::loop();
    }

    sleep_ms(10);

    SysLed::off();

    if (cmd_show) {
        printf("\n");
        printf("dcc_cmd\n");
        printf("\n");
    }

    argv.verbosity(1);

    adc.log_reset(); // logging must be enabled by calling adc.log_init()

    throttle = command.create_throttle(); // default address 3

    if (cmd_show) {
        printf("\n");
        cmd_help(true);
        printf("\n");
    }

    //adc.dbg_loop(21);

    while (true) {

        // If any command is ongoing, see if it has made progress
        if (!(*active)())
            active = &loop_nop; // loop_* returning false means it's done

        // Check for new console input if we're not in the middle of something
        if (active == &loop_nop) {
            // Get console input if available.
            // This might result in a new command.
            int c = stdio_getchar_timeout_us(0);
            if (0 <= c && c <= 255) {
                if (argv.add_char(char(c))) {
                    // newline received, try to process command
                    if (!cmd_try()) {
                        // command not recognized
                        printf("ERROR");
                        if (cmd_show) {
                            printf(": invalid command: ");
                            argv.print();
                            printf("\n");
                            cmd_help(true);
                        }
                        printf("\n");
                    }
                    argv.reset();
                }
            }
        }

        // print anything that might have been logged
        BufLog::loop();

    } // while (true)

    return 0;
}


static bool cmd_try()
{
    assert(argv.argc() > 0);

    if (strcasecmp(argv[0], "L") == 0)
        return loco_try();
    else if (strcasecmp(argv[0], "S") == 0)
        return speed_try();
    else if (strcasecmp(argv[0], "F") == 0)
        return function_try();
    else if (strcasecmp(argv[0], "T") == 0)
        return track_try();
    else if (strcasecmp(argv[0], "C") == 0)
        return cv_try();
    else if (strcasecmp(argv[0], "V") == 0)
        return verbosity_try();
    else if (strcasecmp(argv[0], "A") == 0)
        return address_try();
    else if (strcasecmp(argv[0], "D") == 0)
        return debug_try();
    else
        return false;
}


static void cmd_help(bool verbose)
{
    printf("Commands:\n");
    loco_help(verbose);
    speed_help(verbose);
    function_help(verbose);
    track_help(verbose);
    cv_help(verbose);
    address_help(verbose);
    verbosity_help(verbose);
    debug_help(verbose);
    printf("\n");
    param_help();
}


// L ?          read loco address from current throttle
// L <n>        set loco address in current throttle
// L + <n>      create a new throttle for loco <n> and make it current
// L - <n>      delete throttle for loco <n>

static bool loco_try()
{
    if (argv.argc() == 2) {

        // read loco address from current throttle, or
        // if there is a throttle with the given address,
        //   make it current,
        // or if not,
        //   set loco address in current throttle

        if (strcmp(argv[1], "?") == 0) {
            printf("%d\n", throttle->get_address());
            return true;
        }

        int loco;
        if (str_to_int(argv[1], &loco) && loco >= DccPkt::address_min &&
            loco <= DccPkt::address_max) {
            // if there's already a throttle for this loco, make it current
            // otherwise set the address in the current throttle
            DccThrottle *t = command.find_throttle(loco);
            if (t != nullptr) {
                throttle = t;
            } else {
                throttle->set_address(loco);
                command.restart_throttles();
            }
            printf("OK\n");
            return true;
        }

    } else if (argv.argc() == 3) {

        // create or delete a throttle

        int loco;
        if (str_to_int(argv[2], &loco) && loco >= DccPkt::address_min &&
            loco <= DccPkt::address_max) {

            if (strcmp(argv[1], "+") == 0) {
                throttle = command.create_throttle(loco);
                printf("OK\n");
                command.show();
                return true;
            }

            if (strcmp(argv[1], "-") == 0) {
                throttle = command.delete_throttle(loco);
                printf("OK\n");
                command.show();
                return true;
            }
        }
    }
    return false;
}


static void loco_help(bool verbose)
{
    print_help(verbose, "L ?", "read current address from throttle");
    print_help(verbose, "L <a>",
               "set address in throttle for subsequent operations");
    print_help(verbose, "L + <a>",
               "create throttle for address <a> if it does not already exist");
    print_help(verbose, "L - <a>",
               "delete throttle for address <a> if it exists");
}


static bool speed_try()
{
    if (argv.argc() != 2)
        return false;

    if (strcmp(argv[1], "?") == 0) {
        printf("%d\n", throttle->get_speed());
        return true;
    }

    int speed;
    if (!str_to_int(argv[1], &speed))
        return false;
    if (speed < DccPkt::speed_min || speed > DccPkt::speed_max)
        return false;
    throttle->set_speed(speed);
    printf("OK\n");
    return true;
}


static void speed_help(bool verbose)
{
    print_help(verbose, "S ?", "read speed for current loco");
    print_help(verbose, "S <s>", "set speed for current loco");
}


static bool function_try()
{
    if (argv.argc() == 2) {
        // the only two-token command is "F ?"
        if (strcmp(argv[1], "?") != 0)
            return false;
        // return numbers of functions that are on
        bool first = true;
        for (int i = DccPkt::function_min; i <= DccPkt::function_max; i++) {
            if (throttle->get_function(i)) {
                if (!first)
                    printf(" ");
                printf("%d", i);
                first = false;
            }
        }
        if (!first)
            printf(" ");
        printf("OK\n");
        return true;
    } else if (argv.argc() == 3) {
        int func;
        if (!str_to_int(argv[1], &func))
            return false;
        if (func < DccPkt::function_min || func > DccPkt::function_max)
            return false;
        if (strcmp(argv[2], "?") == 0) {
            printf("%s\n", throttle->get_function(func) ? "ON" : "OFF");
            return true;
        }
        bool setting;
        if (strcasecmp(argv[2], "ON") == 0)
            setting = true;
        else if (strcasecmp(argv[2], "OFF") == 0)
            setting = false;
        else
            return false;
        throttle->set_function(func, setting);
        printf("OK\n");
        return true;
    } else {
        return false;
    }
}


static void function_help(bool verbose)
{
    print_help(verbose, "F ?", "show functions that are on for current loco");
    print_help(verbose, "F <f> ?", "get status of function f for current loco");
    print_help(verbose, "F <f> ON|OFF",
               "set a function for current loco on/off");
}


static bool track_try()
{
    if (argv.argc() != 2)
        return false;

    if (strcmp(argv[1], "?") == 0) {
        printf("%s\n", command.mode() != DccCommand::Mode::OFF ? "ON" : "OFF");
        return true;
    } else if (strcasecmp(argv[1], "ON") == 0) {
        if (command.mode() == DccCommand::Mode::OFF)
            command.set_mode_ops();
        printf("OK\n");
        return true;
    } else if (strcasecmp(argv[1], "OFF") == 0) {
        if (command.mode() == DccCommand::Mode::OPS)
            command.set_mode_off();
        printf("OK\n");
        return true;
    } else {
        return false;
    }
}


static void track_help(bool verbose)
{
    print_help(verbose, "T ?", "get track power status");
    print_help(verbose, "T ON|OFF", "turn track power on/off");
}


/*
CV Access

CV access can be done in either service mode or operations mode. The mode
used depends on the current command station mode (set with the "T" command).
If the command station is powered off, service mode is used. If the command
station is powered on, operations mode is used.

Commands:
C <c> ?        read CV <c>
C <c> <b> ?    read CV <c> bit <b>
C <c> <v>      write CV <c> = <v>
C <c> <b> <v>  write CV <c> bit <b> = <v>
Parameters:
1 <= c <= 1024
0 <= b <= 7
-127 <= v <= +255 for byte writes
0 <= v <= 1 for bit writes
*/

static bool cv_try()
{
    int num_args = argv.argc();

    if (num_args != 3 && num_args != 4)
        return false;

    if (!str_to_int(argv[1], &cv_num_g))
        return false;

    if (cv_num_g < DccPkt::cv_num_min || cv_num_g > DccPkt::cv_num_max)
        return false;

    if (strcmp(argv[num_args - 1], "?") == 0) {
        // read byte or bit
        if (command.mode() == DccCommand::Mode::OPS) {
            // ops mode read using railcom
            if (num_args != 3)
                return false; // there is no ops read-bit command
            // read byte
            throttle->read_cv(cv_num_g);
            ops_throttle_g = throttle;
            active = &loop_ops_cv_read;
        } else if (command.mode() == DccCommand::Mode::OFF) {
            // service mode, read the old-timey way
            adc.log_reset();
            if (num_args == 3) {
                // read byte
                cv_bit_g = -1;
                command.read_cv(cv_num_g);
            } else {
                assert(num_args == 4);
                // read bit
                if (!str_to_int(argv[2], &cv_bit_g))
                    return false;
                if (cv_bit_g < 0 || cv_bit_g > 7)
                    return false;
                command.read_bit(cv_num_g, cv_bit_g);
            }
            active = &loop_svc_cv_read;
        }
        start_us = time_us_64();
        // print OK/ERROR when done
        return true;
    } else {
        // write byte or bit
        if (num_args == 3) {
            // write byte
            if (!str_to_int(argv[2], &cv_val_g))
                return false;
            if (cv_val_g < DccPkt::cv_val_min ||
                cv_val_g > DccPkt::cv_val_max) {
                return false;
            }
            cv_bit_g = -1;
            // use svc mode if not already in ops mode
            if (command.mode() == DccCommand::Mode::OPS) {
                // ops mode
                throttle->write_cv(cv_num_g, cv_val_g);
                printf("OK\n");
            } else {
                // service mode
                assert(command.mode() == DccCommand::Mode::OFF);
                // print OK/ERROR when done
                adc.log_reset();
                command.write_cv(cv_num_g, cv_val_g);
                active = &loop_svc_cv_write;
                start_us = time_us_64();
            }
            return true;

        } else {
            // write bit
            assert(num_args == 4);
            if (!str_to_int(argv[2], &cv_bit_g))
                return false;
            if (cv_bit_g < 0 || cv_bit_g > 7)
                return false;
            if (!str_to_int(argv[3], &cv_val_g))
                return false;
            if (cv_val_g != 0 && cv_val_g != 1)
                return false;

            // use svc mode if not already in ops mode
            if (command.mode() == DccCommand::Mode::OPS) {
                // ops mode
                throttle->write_bit(cv_num_g, cv_bit_g, cv_val_g);
                printf("OK\n");
            } else {
                // service mode
                assert(command.mode() == DccCommand::Mode::OFF);
                // print OK/ERROR when done
                adc.log_reset();
                command.write_bit(cv_num_g, cv_bit_g, cv_val_g);
                active = &loop_svc_cv_write;
                start_us = time_us_64();
            }
            return true;
        }
    }
}


static void cv_help(bool verbose)
{
    print_help(verbose, "C <c> ?", "read cv number <c>");
    print_help(verbose, "C <c> <b> ?", "read cv number <c> bit <b>");
    print_help(verbose, "C <c> <v>", "write cv number <c> with value <v>");
    print_help(verbose, "C <c> <b> 0|1",
               "write cv number <n> bit <b> with 0/1");
}


static bool verbosity_try()
{
    if (argv.argc() != 3) {
        // wrong number of arguments
        return false;
    }

    if (strcasecmp(argv[1], "C") == 0) {
        if (strcasecmp(argv[2], "ON") == 0) {
            cmd_show = true;
            argv.verbosity(1);
            printf("OK\n");
            return true;
        } else if (strcasecmp(argv[2], "OFF") == 0) {
            cmd_show = false;
            argv.verbosity(0);
            printf("OK\n");
            return true;
        } else if (strcasecmp(argv[2], "?") == 0) {
            printf("%s\n", cmd_show ? "ON" : "OFF");
            return true;
        }
        // argv[2] unrecognized
        return false;
    }

    if (strcasecmp(argv[1], "D") == 0) {
        if (strcasecmp(argv[2], "ON") == 0) {
            command.show_dcc(true);
            printf("OK\n");
            return true;
        } else if (strcasecmp(argv[2], "OFF") == 0) {
            command.show_dcc(false);
            printf("OK\n");
            return true;
        } else if (strcmp(argv[2], "?") == 0) {
            printf("%s\n", command.show_dcc() ? "ON" : "OFF");
            return true;
        }
        // argv[2] unrecognized
        return false;
    }

    if (strcasecmp(argv[1], "R") == 0) {
        if (strcasecmp(argv[2], "ON") == 0) {
            command.show_railcom(true);
            printf("OK\n");
            return true;
        } else if (strcasecmp(argv[2], "OFF") == 0) {
            command.show_railcom(false);
            printf("OK\n");
            return true;
        } else if (strcmp(argv[2], "?") == 0) {
            printf("%s\n", command.show_railcom() ? "ON" : "OFF");
            return true;
        }
        // argv[2] unrecognized
        return false;
    }

    if (strcasecmp(argv[1], "S") == 0) {
        if (strcasecmp(argv[2], "ON") == 0) {
            command.show_rc_speed(true);
            printf("OK\n");
            return true;
        } else if (strcasecmp(argv[2], "OFF") == 0) {
            command.show_rc_speed(false);
            printf("OK\n");
            return true;
        } else if (strcmp(argv[2], "?") == 0) {
            printf("%s\n", command.show_rc_speed() ? "ON" : "OFF");
            return true;
        }
        // argv[2] unrecognized
        return false;
    }

    // argv[1] unrecognized
    return false;
}


static void verbosity_help(bool verbose)
{
    print_help(verbose, "V C ON|OFF", "show more command feedback");
    print_help(verbose, "V D ON|OFF", "show DCC packets or not");
    print_help(verbose, "V R ON|OFF", "show RailCom packets or not");
    print_help(verbose, "V S ON|OFF", "show RailCom reported speed or not");
    print_help(verbose, "V C|D|R ?", "get show setting");
}


// All paths with expected output:
//
// Always two tokens; first is "A" and second is "?" to read or an integer to
// write.
//
// Address is short if it is <= 127
// Address is long if it is > 127
// Can't write a short address that uses the long-address registers.
//
// To write:
//  Here:
//      set address_g with address to write (long or short)
//      if short:
//          start write of address here:        cv_num_g = 1,  cv_val_g =
//          address_g
//      if long:
//          start write of address_lo here:     cv_num_g = 18, cv_val_g =
//          address_g & 0xff
//  In loop_svc_address_write:
//      if cv_num_g is 1 (address):
//          start clearing of cv29[5]:          cv_num_g = 29, cv_val_g = 0
//      else if cv_num_g is 18 (address_lo):
//          start write of address_hi:          cv_num_g = 17, cv_val_g =
//          (address >> 8) | 0xc0
//      else if cv_num_g is 17 (address_hi):
//          start setting of cv29[5]:           cv_num_g = 29, cv_val_g = 1
//      else if cv_num_g is 29 (config):
//          done (success)
// To read:
//  Here:
//      start read of cv29[5]:                  cv_num_g = 29
//  In loop_svc_addres_read:
//      if cv_num_g is 29 (config):
//          if cv29[5]=0, start read of cv1:    cv_num_g = 1
//          if cv29[5]=1, start read of cv18:   cv_num_g = 18
//      else if cv_num_g is 1 (address):
//          done (success), address_g = value
//      else if cv_num_g is 18 (address_lo):
//          address_g = value
//          start read of cv17:                 cv_num_g = 17
//      else if cv_num_g is 17 (address_hi):
//          done (success), address_g = address_g | (value & 0x3f) << 8

static bool address_try()
{
    // address_g is global

    if (argv.argc() != 2)
        return false;

    if (command.mode() != DccCommand::Mode::OFF)
        return false;

    if (strcmp(argv[1], "?") == 0) {
        // Read CV29, and bit 5 tells us if it's a short or long address
        // Bit 5 = 0: short address; read CV1
        // Bit 5 = 1: long address; read CV17 and CV18
        active = &loop_svc_address_read;
        cv_num_g = DccCv::config;
        command.read_bit(cv_num_g, 5);
    } else {
        if (!str_to_int(argv[1], &address_g))
            return false;
        if (address_g < DccPkt::address_min ||
            address_g > DccPkt::address_max) {
            return false;
        }
        // Short address is <= 127
        // Short address: write CV1, then clear CV29 bit 5
        // Long address: write CV18 and CV17, then set CV29 bit 5
        // Start the first write (CV1 or CV17) and let loop_svc_address_write
        // figure out what to do next (based on cv_num_g).
        if (address_g <= 127) {
            cv_num_g = DccCv::address;
            cv_val_g = address_g;
        } else {
            cv_num_g = DccCv::address_lo;
            cv_val_g = address_g & 0xff;
        }
        active = &loop_svc_address_write;
        command.write_cv(cv_num_g, cv_val_g);
    }

    start_us = time_us_64();
    // print OK/ERROR when done
    return true;
}


static void address_help(bool verbose)
{
    print_help(verbose, "A ?", "read address from loco (long or short)");
    print_help(verbose, "A <a>", "write address to loco (long or short)");
}


// Debug ADC (dump log)
// D A

static bool debug_try()
{
    if (argv.argc() != 2)
        return false;

    if (adc.logging()) {
        if (strcasecmp(argv[1], "A") == 0) {
            adc.log_show();
            adc.log_reset();
            return true;
        }
    }

    return false;
}


static void debug_help(bool verbose)
{
    if (adc.logging()) {
        print_help(verbose, "D A", "dump ADC log");
    }
}


static void param_help()
{
    int n;
    constexpr int tab_col = 20;

    printf("Parameters:\n");

    n = printf("%d <= a <= %d", DccPkt::address_min, DccPkt::address_max);
    while (++n < tab_col) {
        printf(" ");
    }
    printf("loco address\n");

    n = printf("%d <= s <= %d", DccPkt::speed_min, DccPkt::speed_max);
    while (++n < tab_col) {
        printf(" ");
    }
    printf("loco speed\n");

    n = printf("%d <= f <= %d", DccPkt::function_min, DccPkt::function_max);
    while (++n < tab_col) {
        printf(" ");
    }
    printf("function number\n");

    n = printf("%d <= c <= %d", DccPkt::cv_num_min, DccPkt::cv_num_max);
    while (++n < tab_col) {
        printf(" ");
    }
    printf("cv number\n");

    n = printf("%d <= v <= %d", DccPkt::cv_val_min, DccPkt::cv_val_max);
    while (++n < tab_col) {
        printf(" ");
    }
    printf("cv value\n");

    n = printf("%d <= b <= %d", 0, 7);
    while (++n < tab_col) {
        printf(" ");
    }
    printf("bit number\n");
}


static void print_help(bool verbose, const char *help_short,
                       const char *help_long)
{
    if (verbose) {
        printf("%-20s", help_short);
        printf(help_long);
    } else {
        printf(help_short);
    }
    printf("\n");
}


// the "active" function when nothing needs doing
static bool loop_nop()
{
    return true;
}


static bool loop_ops_cv_read()
{
    bool result;
    uint8_t value;

    // In ops mode, it is the throttle that knows when it is done.
    // XXX This assumes only one throttle doing an ops mode op at once.
    assert(ops_throttle_g != nullptr);
    if (!ops_throttle_g->ops_done(result, value))
        return true; // keep going

    uint32_t op_ms = usec_to_msec(time_us_64() - start_us);

    // it's always a byte read (there is no ops mode bit read)
    if (result) {
        printf("%u", uint(value));
        if (cmd_show)
            printf(" (0x%02x) in %lu ms", uint(value), op_ms);
        printf("\n");

    } else {
        printf("ERROR");
        if (cmd_show)
            printf(" in %lu ms", op_ms);
        printf("\n");
    }

    ops_throttle_g = nullptr;

    return false; // done!
}


static bool loop_svc_cv_read()
{
    bool result;
    uint8_t value;
    if (!command.svc_done(result, value))
        return true; // keep going

    uint32_t op_ms = usec_to_msec(time_us_64() - start_us);

    if (result) {
        printf("%u", uint(value));
        if (cmd_show) {
            if (cv_bit_g < 0 || cv_bit_g > 7)
                printf(" (0x%02x)", uint(value)); // byte read
            printf(" in %lu ms", op_ms);
        }
        printf("\n");
    } else {
        printf("ERROR");
        if (cmd_show)
            printf(" in %lu ms", op_ms);
        printf("\n");
    }

    return false; // done!
}


static bool loop_svc_cv_write()
{
    bool result;
    if (!command.svc_done(result))
        return true; // keep going

    uint32_t op_ms = usec_to_msec(time_us_64() - start_us);

    if (result) {
        printf("OK");
        if (cmd_show)
            printf(" in %lu ms", op_ms);
        printf("\n");
    } else {
        printf("ERROR");
        if (cmd_show)
            printf(" in %lu ms", op_ms);
        printf("\n");
    }

    return false; // done!
}


static bool loop_svc_address_read()
{
    bool result;
    uint8_t value;
    if (!command.svc_done(result, value))
        return true; // keep going

    uint32_t op_ms = usec_to_msec(time_us_64() - start_us);

    if (!result) {
        printf("ERROR");
        if (cmd_show) {
            printf(" reading cv%d", cv_num_g);
            if (cv_num_g == DccCv::config)
                printf("[5]");
            printf(" in %lu ms", op_ms);
        }
        printf("\n");
        return false; // done!
    }

    if (cv_num_g == DccCv::config) {
        // done reading the config bit
        assert(value == 0 || value == 1);
        if (cmd_show) {
            printf("read cv%d[5] = %u in %lu ms\n", cv_num_g, uint(value),
                   op_ms);
        }
        if (value == 0)
            cv_num_g = DccCv::address; // short address, read it
        else
            cv_num_g = DccCv::address_lo; // long address, read lower half
        command.read_cv(cv_num_g);
        start_us = time_us_64();
        return true; // keep going
    } else {
        if (cmd_show) {
            printf("read cv%d = %u (0x%02x) in %lu ms\n", cv_num_g, uint(value),
                   uint(value), op_ms);
        }
        if (cv_num_g == DccCv::address) {
            // short address, done
            address_g = uint(value);
            printf("%d", address_g);
            if (cmd_show)
                printf(" (short)");
            printf("\n");
            return false;
        } else if (cv_num_g == DccCv::address_lo) {
            // long address: save address_lo and read address_hi
            address_g = uint(value);
            cv_num_g = DccCv::address_hi;
            command.read_cv(cv_num_g);
            start_us = time_us_64();
            return true; // keep going
        } else {
            assert(cv_num_g == DccCv::address_hi);
            // long address, done
            address_g |= (int(value & ~0xc0) << 8);
            printf("%d", address_g);
            if (cmd_show)
                printf(" (long)");
            printf("\n");
            return false;
        }
    }
}


static bool loop_svc_address_write()
{
    bool result;
    if (!command.svc_done(result))
        return true; // keep going

    // one of the writes has finished

    uint32_t op_ms = usec_to_msec(time_us_64() - start_us);

    assert(DccPkt::address_min <= address_g &&
           address_g <= DccPkt::address_max);
    assert(cv_num_g == DccCv::address || cv_num_g == DccCv::address_lo ||
           cv_num_g == DccCv::address_hi || cv_num_g == DccCv::config);

    if (!result) {
        printf("ERROR");
        if (cmd_show) {
            printf("ERROR writing cv%d", cv_num_g);
            if (cv_num_g == DccCv::config)
                printf("[5] = %u", uint(cv_val_g));
            else
                printf(" = %u (0x%02x)", uint(cv_val_g), uint(cv_val_g));
            printf(" in %lu ms", op_ms);
        }
        printf("\n");
        return false; // done!
    }

    if (cmd_show) {
        printf("wrote cv%d", cv_num_g);
        if (cv_num_g == DccCv::config)
            printf("[5] = %u", uint(cv_val_g));
        else
            printf(" = %u (0x%02x)", uint(cv_val_g), uint(cv_val_g));
        printf(" in %lu ms\n", op_ms);
    }

    if (cv_num_g == DccCv::address) {
        // short address, and just wrote CV1 (address)
        // clear CV29 bit 5
        cv_num_g = DccCv::config;
        cv_val_g = 0; // only used for an error message if needed
        command.write_bit(cv_num_g, 5, 0);
        start_us = time_us_64();
        return true; // keep going
    } else if (cv_num_g == DccCv::address_lo) {
        cv_num_g = DccCv::address_hi;
        cv_val_g = (address_g >> 8) | 0xc0;
        command.write_cv(cv_num_g, cv_val_g);
        start_us = time_us_64();
        return true; // keep going
    } else if (cv_num_g == DccCv::address_hi) {
        // set CV29 bit 5
        cv_num_g = DccCv::config;
        cv_val_g = 1; // only used for an error message if needed
        command.write_bit(cv_num_g, 5, 1);
        start_us = time_us_64();
        return true; // keep going
    } else {
        assert(cv_num_g == DccCv::config);
    }

    cv_num_g = DccPkt::cv_num_inv;
    cv_val_g = DccPkt::cv_val_inv;

    printf("OK");
    if (cmd_show) {
        if (address_g <= 127)
            printf(" (short)");
        else
            printf(" (long)");
    }
    printf("\n");

    return false; // done!
}
