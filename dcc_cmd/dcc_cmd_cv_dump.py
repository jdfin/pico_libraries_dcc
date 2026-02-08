import time
import serial as ps
import re


# railcom page:
# CV 31 = 0, CV 32 = 255
# manufacturer ID: CVs 257-260
# product ID: CVs 261-264
# serial number: CVs 265-268
# production date: CVs 269-272

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

port = ps.Serial('/dev/ttyACM1', timeout=5)

verbose = True

time.sleep(1)


def flush_in():
    time.sleep(0.01)
    while port.in_waiting > 0:
        x = port.read(port.in_waiting)
        time.sleep(0.01)


def run_cmd(cmd, exp=None):
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
    if correct and (exp is not None):
        if verbose:
            correct = rsp.startswith(exp)
        else:
            correct = (rsp == exp)
    #print(f'CMD: "{cmd}" RSP: "{rsp}" EXP: "{exp}" CORRECT: {correct}')
    print(f'"{cmd}" --> "{rsp}"')
    if correct:
        return rsp
    else:
        return ''


def read_cv(num, offset=False):
    rsp = run_cmd(f'C {num} ?')
    val = re.split(r'[ ()]+', rsp)
    if offset:
        num -= 257
    #print(f'{num} {val[0]} {val[1]}')


run_cmd('T ON', 'OK')
run_cmd('V C ON', 'OK')
run_cmd('C 8 8', 'OK')
for cv in range(1, 257):
    read_cv(cv)

print('\nDefault Page:\n')
run_cmd('C 31 16', 'OK')
run_cmd('C 32 0', 'OK')
for cv in range(257, 513):
    read_cv(cv)

print('\nRailCom Page:\n')
run_cmd('C 31 0', 'OK')
run_cmd('C 32 255', 'OK')
for cv in range(257, 513):
    read_cv(cv, True)


port.close()
