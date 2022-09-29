import sys
import datetime, time
import argparse
import re
import shutil
import struct
import os

from os.path import join, dirname
from pathlib import Path

FLASH_PAGE_COUNT = (128)
FLASH_PAGE_SIZE = (1024)
FLASH_TOTAL_SIZE = (FLASH_PAGE_COUNT * FLASH_PAGE_SIZE)
BOOT_CODE_SIZE = (20 * 1024)
MAC_PARA_SIZE = (1 * 1024)
PUBLIC_PARA_SIZE = (1 * 1024)
APP_PARA_SIZE = (1 * 1024)
FLASH_BASE = (0)
FLASH_BOOT_CODE = (FLASH_BASE)
FLASH_MODULE_PARA = (FLASH_BOOT_CODE + BOOT_CODE_SIZE)
FLASH_PUBLIC_PARA = (FLASH_MODULE_PARA + MAC_PARA_SIZE)
FLASH_APP_PARA = (FLASH_PUBLIC_PARA + PUBLIC_PARA_SIZE)
FLASH_APP = (FLASH_APP_PARA + APP_PARA_SIZE)
APP_VARSIONS_SIZE = 32

_VERSION = "V1.0.0"


def encode_10_to_36(num, bit_num=1):
    encode = bytearray(bit_num)
    for i in range(bit_num):
        mod = num % 36
        if mod < 10:
            encode[bit_num - i - 1] = ord(str(int(mod)))
        else:
            encode[bit_num - i - 1] = ord('A') + (mod - 10)
        num //= 36
    return bytes(encode)


def get_mac_info(id, sn, p1, p2, p3, p4, hw):
    module_id = encode_10_to_36(id, 2)
    t = time.localtime()
    year = encode_10_to_36(t.tm_year - 2019)
    mon = encode_10_to_36(t.tm_mon)
    day = encode_10_to_36(t.tm_mday)
    hour = encode_10_to_36(t.tm_hour, 2)
    minute = encode_10_to_36(t.tm_min, 2)
    sec = encode_10_to_36(t.tm_sec, 2)
    random = encode_10_to_36(0, 4)
    mac = struct.pack('2sccc2s2s2sB4siiiiiB', module_id, year, mon, day,
                        hour, minute, sec, 0, random, sn,
                        p1, p2, p3, p4, hw)
    return mac


def load_boot_file(boot_path):
    try:
        with open(boot_path, 'rb') as f:
            data = f.read()
            f.close()
            return data
    except:
        raise RuntimeError("failed to load bootloader: {}".format(boot_path))


def load_app_file(app_path):
    try:
        with open(app_path, 'rb') as f:
            data = f.read()
            f.close()
            return data
    except:
        raise RuntimeError("failed to load app: {}".format(app_path))


def write_boot_file(file, boot):
    file.write(boot)


def write_module_parm(file, mac):
    file.seek(FLASH_MODULE_PARA, 0)
    file.write(bytes([0xff for i in range(MAC_PARA_SIZE)]))
    file.seek(FLASH_MODULE_PARA, 0)
    file.write(mac)


def write_public_parm(file):
    param = bytes([0xff, 0xaa])
    file.seek(FLASH_PUBLIC_PARA, 0)
    file.write(bytes([0xff for i in range(MAC_PARA_SIZE)]))
    file.seek(FLASH_PUBLIC_PARA, 0)
    file.write(param)


def write_app_parm(file, version):
    b = bytearray(version, encoding="utf-8")
    l = len(b)
    param = bytearray([0xff for i in range(MAC_PARA_SIZE)])

    if l >= 32:
        for i in range(31):
            param[i] = b[i]
        param[31] = 0
    else:
        for i in range(l):
            param[i] = b[i]
        param[l] = 0

    file.seek(FLASH_APP_PARA, 0)
    file.write(param)


def write_app_file(file, app):
    file.seek(FLASH_APP, 0)
    file.write(app)


def main(argv=None):
    parser = argparse.ArgumentParser("Package firmware for Snapmaker 2.0 -- V{}".format(_VERSION))

    parser.add_argument('-b', '--boot',
                        help="specify bootloader",
                        type=str,
                        default=None)

    parser.add_argument('-a', '--app',
                        help="specify raw binary image of app",
                        type=str,
                        default=None)

    parser.add_argument('-v', '--ver',
                        help="specify version of app fw",
                        type=str,
                        default=None)

    parser.add_argument('-hw', '--hw_ver',
                        help="specify hardware version number",
                        type=int,
                        default=0xff)

    parser.add_argument('-s', '--sn',
                        help="specify serial number",
                        type=int,
                        default=1000)

    parser.add_argument('-i', '--id',
                        help="specify device id of module",
                        type=int,
                        default=0xffff)

    parser.add_argument('-p1', '--param1',
                        help="specify the first private parameter",
                        type=int,
                        default=0)

    parser.add_argument('-p2', '--param2',
                        help="specify the second private parameter",
                        type=int,
                        default=0)

    parser.add_argument('-p3', '--param3',
                        help="specify the third private parameter",
                        type=int,
                        default=0)

    parser.add_argument('-p4', '--param4',
                        help="specify the fourth private parameter",
                        type=int,
                        default=0)

    parser.add_argument('-o', '--output',
                        help="specify the directory of output file",
                        type=str,
                        default=None)

    parser.add_argument('-p', '--proj',
                        help="specify the directory of priject",
                        type=str,
                        default=None)

    if argv != None:
        args = parser.parse_args(argv)
    else:
        args = parser.parse_args()

    if args.proj is None:
        raise RuntimeError("must specify project file directory")

    if args.output is None:
        args.output = args.proj

    boot = load_boot_file(args.boot)
    if boot is None:
        raise RuntimeError("no content in bootloader")

    app = load_app_file(args.app)
    if app is None:
        raise RuntimeError("no content in app")

    mac = get_mac_info(args.id, args.sn, args.param1, args.param2, args.param3, args.param4, args.hw_ver)
    if mac is None:
        raise RuntimeError("failed to create mac info")

    version = None
    if args.ver == None and args.proj != None:
        with open(join(args.proj, 'Marlin', 'src', 'configuration.h'), 'r', encoding='utf-8') as version_file:
            lines = version_file.readlines()

        pattern = r"v\d+\.\d+\.\d+"
        for line in lines:
            match_obj = re.search(pattern, line, re.I)
            if match_obj:
                print("got version: {}".format(match_obj[0]))
                version = match_obj[0]
                break

    if version == None:
        raise RuntimeError("No version specify!")

    date  = datetime.datetime.today().strftime('%Y%m%d')

    of = "SM_MODULE_BOOT_APP_{}_{}.bin".format(version, date)
    file_path = join(args.output, of)

    print("device id: {}".format(args.id))
    print("serial number: {:#X}".format(args.sn))
    print("fw version: {}".format(version))
    print("hw version: {:#X}".format(args.hw_ver))
    print("bootloader: {}".format(args.boot))
    print("app: {}".format(args.app))
    print("output: {}".format(file_path))

    try:
        with open(file_path, 'wb') as f:
            write_boot_file(f, boot)
            write_module_parm(f, mac)
            write_public_parm(f)
            write_app_parm(f, version)
            write_app_file(f, app)
            f.close()
    except:
        raise RuntimeError("failed to create image")

if __name__ == "__main__":
    main()
