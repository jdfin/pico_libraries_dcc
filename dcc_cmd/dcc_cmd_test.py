import time
import serial as ps

verbosity_tests = [
    [ 'V', 'ERROR' ],           # argc != 3
    [ 'V C', 'ERROR' ],         # argc != 3
    [ 'V X ON', 'ERROR' ],      # argv[1] invalid
    [ 'V C 0', 'ERROR' ],       # argv[2] invalid
]

track_tests = [
    [ 'T', 'ERROR' ],           # argc != 2
    [ 'T X', 'ERROR' ],         # argv[1] invalid
    [ 'T 2', 'ERROR' ],         # argv[1] invalid
    [ 'T ON 0', 'ERROR' ],      # argc != 2
    [ 'T ON', 'OK' ],
    [ 'T ?', 'ON' ],
    [ 'T OFF', 'OK' ],
    [ 'T ?', 'OFF' ],
]

loco_tests = [
    # argc < 2
    [ 'L', 'ERROR' ],           # argc < 2
    # argc == 2
    [ 'L X', 'ERROR' ],         # argv[1] not a number
    [ 'L 20000', 'ERROR' ],     # argv[1] out of range
    [ 'L 3', 'OK' ],
    [ 'L ?', '3' ],
    [ 'L 2265', 'OK' ],
    [ 'L ?', '2265' ],
    [ 'L 3', 'OK' ],
    [ 'L ?', '3' ],
    # argc == 3
    [ 'L 3 3', 'ERROR' ],       # argv[1] invalid
    [ 'L + 4', 'OK' ],
    [ 'L - 4', 'OK' ],
    [ 'L - 4', 'OK' ],
    # argc > 3
    [ 'L 3 4 5', 'ERROR' ],     # argc > 3
]

speed_tests = [
    # argc < 2
    [ 'S', 'ERROR' ],           # argc < 2
    # argc == 2
    [ 'S X', 'ERROR' ],         # argv[1] invalid
    [ 'S 200', 'ERROR' ],       # speed out of range
    [ 'S -200', 'ERROR' ],      # speed out of range
    [ 'S 50', 'OK' ],
    [ 'S -50', 'OK' ],
    [ 'S 0', 'OK' ],
    [ 'S ?', '0' ],
    # argc > 2
    [ 'S 3 X', 'ERROR' ],       # argc > 2
]

function_tests = [
    # argc < 2
    [ 'F', 'ERROR' ],           # argc < 2
    # argc == 2
    [ 'F 0', 'ERROR' ],         #
    [ 'F X', 'ERROR' ],         #
    # argc == 3
    [ 'F -1 OFF', 'ERROR' ],    # function number out of range
    [ 'F 0 OFF', 'OK' ],
    [ 'F 1 OFF', 'OK' ],
    [ 'F 2 OFF', 'OK' ],
    [ 'F 3 OFF', 'OK' ],
    [ 'F 4 OFF', 'OK' ],
    [ 'F 5 OFF', 'OK' ],
    [ 'F 6 OFF', 'OK' ],
    [ 'F 7 OFF', 'OK' ],
    [ 'F 8 OFF', 'OK' ],
    [ 'F 9 OFF', 'OK' ],
    [ 'F 10 OFF', 'OK' ],
    [ 'F 11 OFF', 'OK' ],
    [ 'F 12 OFF', 'OK' ],
    [ 'F 13 OFF', 'OK' ],
    [ 'F 14 OFF', 'OK' ],
    [ 'F 15 OFF', 'OK' ],
    [ 'F 16 OFF', 'OK' ],
    [ 'F 17 OFF', 'OK' ],
    [ 'F 18 OFF', 'OK' ],
    [ 'F 19 OFF', 'OK' ],
    [ 'F 20 OFF', 'OK' ],
    [ 'F 21 OFF', 'OK' ],
    [ 'F 22 OFF', 'OK' ],
    [ 'F 23 OFF', 'OK' ],
    [ 'F 24 OFF', 'OK' ],
    [ 'F 25 OFF', 'OK' ],
    [ 'F 26 OFF', 'OK' ],
    [ 'F 27 OFF', 'OK' ],
    [ 'F 28 OFF', 'OK' ],
    [ 'F 29 OFF', 'OK' ],
    [ 'F 30 OFF', 'OK' ],
    [ 'F 31 OFF', 'OK' ],
    [ 'F 32 OFF', 'ERROR' ],    # function number out of range
    [ 'F X ON', 'ERROR' ],      #
    [ 'F ? ON', 'ERROR' ],      #
    [ 'F ? X', 'ERROR' ],       #
    [ 'F 0 ON', 'OK' ],
    [ 'F 0 ?', 'ON' ],
    [ 'F 0 OFF', 'OK' ],
    [ 'F 0 ?', 'OFF' ],
    [ 'F 0 X', 'ERROR' ],       # argv[2] invalid
    # argc > 3
    [ 'F 0 1 ON', 'ERROR' ],    # argc > 3
]

cv_tests = [
#   [ 'C', 'ERROR' ],           # argc < 3
#   [ 'C 1', 'ERROR' ],         # argc < 3
#   [ 'C 0 0', 'ERROR' ],       # cv_num out of range
#   [ 'C 1025 0', 'ERROR' ],    # cv_num out of range

#   [ 'T OFF', 'OK' ],          # track off (svc mode)

#   [ 'C 3 X', 'ERROR' ],       # argv[2] invalid
#   [ 'C 3 -300', 'ERROR' ],    # argv[2] out of range
#   [ 'C 3 300', 'ERROR' ],     # argv[2] out of range
#   [ 'C 8 8', 'OK' ],          # reset
#   [ 'C 8 ?', '151' ],         # mfg id = ESU
#   [ 'C 8 7 ?', '1' ],         # cv8 bit 7
#   [ 'C 8 6 ?', '0' ],         # cv8 bit 6

#   [ 'T ON', 'OK' ],           # track on (ops mode)

#   [ 'C 3 X', 'ERROR' ],       # argv[2] invalid
#   [ 'C 3 -300', 'ERROR' ],    # argv[2] out of range
#   [ 'C 3 300', 'ERROR' ],     # argv[2] out of range
#   [ 'C 8 8', 'OK' ],          # reset
#   [ 'C 8 ?', '151' ],         # using railcom
#   [ 'C 8 7 ?', 'ERROR' ],     # no ops mode read bit

    [ 'T OFF', 'OK' ],          # track off (svc mode)
    [ 'C 8 8', 'OK' ],          # reset loco to adrs 3
    [ 'T ON', 'OK' ],           # track on (ops mode)
    [ 'L 3', 'OK' ],            # expected loco
    [ 'C 8 ?', '151' ],         #
    [ 'L 4', 'OK' ],            # nonexistent loco
    [ 'C 8 ?', 'ERROR' ],       #

    [ 'T OFF', 'OK' ],          # leave track off
]

address_tests = [
    [ 'T ?', 'OFF' ], # track must be off (svc mode)
    [ 'A', 'ERROR' ], # argc != 2
    [ 'A X', 'ERROR' ], # addr not an integer
    [ 'A 0', 'ERROR' ], # addr out of range
    [ 'A 99999', 'ERROR' ], # addr out of range
    [ 'A 3', 'OK' ], # set addr
    [ 'A ?', '3' ], # get addr
    [ 'A 2265', 'OK' ], # set addr
    [ 'A ?', '2265' ], # get addr
    [ 'A 3', 'OK' ], # set addr
    [ 'A ?', '3' ], # get addr
]

railcom_tests = [
    [ 'C 8 8', 'OK' ],
    [ 'C 31 0', 'OK' ],
    [ 'C 32 255', 'OK' ],
    # manufacturer ID
    [ 'C 257 ?', '' ], # 0
    [ 'C 258 ?', '' ], # 1
    [ 'C 259 ?', '' ], # 2
    [ 'C 260 ?', '' ], # 3
    # product ID
    [ 'C 261 ?', '' ], # 4
    [ 'C 262 ?', '' ], # 5
    [ 'C 263 ?', '' ], # 6
    [ 'C 264 ?', '' ], # 7
    # serial number
    [ 'C 265 ?', '' ], # 8
    [ 'C 266 ?', '' ], # 9
    [ 'C 267 ?', '' ], # 10
    [ 'C 268 ?', '' ], # 11
    # production date
    [ 'C 269 ?', '' ], # 12
    [ 'C 270 ?', '' ], # 13
    [ 'C 271 ?', '' ], # 14
    [ 'C 272 ?', '' ], # 15
]

# MP1130:
# mfg_id:  00_00_00_97 = 151, ok
# prd_id:  02_00_00_93 = 33,554,579 maybe 147
# ser_num: df_fa_b4_ac = 3,757,749,420
# prd_dat: 27_d2_e4_be = 668,132,542 sec = ~21.17 years, ok

# SP2265:
# mfg_id:  00_00_00_97
# prd_id:  02_00_00_9e
# ser_num: f9_e8_67_69
# prd_dat: 2c_64_8d_8a

# UP 852:
# mfg_id:  00_00_00_97 = 151, ok
# prd_id:  02_00_00_db
# ser_num: df_f5_f5_d4
# prd_dat: 2b_2f_d0_0f

tests = [
    #verbosity_tests,
    #track_tests,
    #loco_tests,
    #speed_tests,
    #function_tests,
    #cv_tests,
    #address_tests,
    railcom_tests,
]

#port = ps.Serial('COM10', timeout=5)
port = ps.Serial('/dev/ttyACM1', timeout=5)

verbose = True
pass_count = 0
fail_count = 0

time.sleep(1)


def flush_in():
    time.sleep(0.1)
    while port.in_waiting > 0:
        x = port.read(port.in_waiting)
        time.sleep(0.1)


def one_test(cmd, exp):
    flush_in()
    correct = True
    port.write((cmd + '\r\n').encode('utf-8'))
    if verbose:
        echo = port.readline().decode('utf-8').replace('\r', '').replace('\n', '')
        if echo != cmd:
            correct = False
    else:
        echo = ''
    rsp = port.readline().decode('utf-8').replace('\r', '').replace('\n', '')
    if correct:
        if verbose:
            correct = rsp.startswith(exp)
        else:
            correct = (rsp == exp)
    global pass_count, fail_count
    if correct:
        pass_count += 1
        print(f"{cmd:8} -->  {echo:8} -->  {rsp:48} >>>>> PASS")
    else:
        fail_count += 1
        print(f"{cmd:8} -->  {echo:8} -->  {rsp:48} >>>>> FAIL: expected '" + exp + "'")
    return correct, echo, rsp


def one_group(group):
    for t in group:
        flush_in()
        one_test(t[0], t[1])


for group in tests:
    # run in both verbose and non-verbose modes
    one_test('V C ON', 'OK')
    verbose = True
    one_group(group)
    one_test('V C OFF', 'OK')
    verbose = False
    one_group(group)

one_test('V C ON', 'OK')
verbose = True

print(f"Passed: {pass_count}, Failed: {fail_count}")

port.close()