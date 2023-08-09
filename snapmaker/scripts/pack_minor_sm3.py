#!/usr/bin/env python3
# -*- coding: utf-8 -*-

' fms '
__author__ = '747'

from enum import Enum
import argparse
from pathlib import Path
import time, datetime
import ntpath
from os.path import join
import os
import re
import struct

# type id in major image
TYPE_MAIN_CONTROLLER = 0
TYPE_EXTERNAL_MODULE = 1
TYPE_SCREEN_MODULE = 2

def crc32(data):
  checksum = 0
  l = len(data)

  for j in range(0, (int)(l / 2) * 2, 2):
    checksum += ((data[j]<<8) | data[j+1])

  if (l % 2):
    checksum += data[l - 1]

  checksum = checksum & 0xFFFFFFFF
  checksum = ~checksum
  checksum = checksum & 0xFFFFFFFF
  return checksum

def crc16(data):
  checksum = 0
  l = len(data)

  for j in range(0, (int)(l / 2) * 2, 2):
    checksum += ((data[j]<<8) | data[j+1])

  if (l % 2):
    checksum += data[l - 1]

  while checksum > 0xFFFF:
    checksum = ((checksum >> 16) & 0xFFFF) + (checksum & 0xFFFF)

  checksum = checksum & 0xFFFF;
  checksum = ~checksum
  checksum = checksum & 0xFFFF;

  return checksum


def append_head(output, type_, offset, filename):
    file_path = os.path.join(os.getcwd(), filename)
    size = os.path.getsize(file_path)

    output.write(struct.pack('>b', type_))
    output.write(struct.pack('>i', offset))
    output.write(struct.pack('>i', size))

    return size


def append_body(output, filename):
    file_path = os.path.join(os.getcwd(), filename)

    with open(file_path, 'rb') as f:
        output.write(f.read())


def pack_major_image(controller=None, module=None, screen=None, version=None):
    count = 0
    date  = datetime.datetime.today().strftime('%Y%m%d')
    version_pattern = r"V\d+\.\d+\.\d+"

    if isinstance(module, Path):
        print("module path: {}".format(module.absolute()))
        count += 1
        cur_dir = module.parent
        if version == None:
            version = re.search(version_pattern, module.name)[0]

    if isinstance(controller, Path):
        print("controller path: {}".format(controller.absolute()))
        count += 1
        cur_dir = controller.parent
        if version == None:
            version = re.search(version_pattern, controller.name)[0]

    if isinstance(screen, Path):
        print("screen path: {}".format(screen.absolute()))
        count += 1
        cur_dir = screen.parent
        if version == None:
            version = re.search(version_pattern, screen.name)[0]

    full_version = "SM3_EM_HMI_{}_{}".format(version, date)

    major_image = cur_dir.joinpath(full_version + '.bin')
    if major_image.exists():
        if major_image.with_suffix('.bin.old').exists():
            os.remove(major_image.with_suffix('.bin.old'))
        os.rename(major_image, major_image.with_suffix('.bin.old'))

    full_version = full_version.encode('ASCII')
    for i in range(32 - len(full_version)):
        full_version += b'\0'

    if count == 0:
        raise RuntimeError("Please specify minor image to be packaged!")

    print("Major image: {}".format(major_image))
    header_size = 39 + 9 * count
    with open(major_image, 'wb') as f:
        # Header (39 bytes)
        # - Length (2 bytes)
        # - Version (32 bytes)
        # - Flag (4 bytes)
        # - Count (1 byte)
        f.write(struct.pack('>h', header_size))  # Length 2 Bytes

        f.write(full_version)  # version 32 bytes

        f.write(struct.pack('>i', 0))  # flag 4 bytes
        f.write(struct.pack('>b', count))  # count

        offset = header_size

        # Module Header (9 bytes for each module)
        if isinstance(module, Path):
            offset += append_head(f, TYPE_EXTERNAL_MODULE, offset, module)
        if isinstance(controller, Path):
            offset += append_head(f, TYPE_MAIN_CONTROLLER, offset, controller)
        if isinstance(screen, Path):
            offset += append_head(f, TYPE_SCREEN_MODULE, offset, screen)

        # Module Body
        if isinstance(module, Path):
            append_body(f, module)
        if isinstance(controller, Path):
            append_body(f, controller)
        if isinstance(screen, Path):
            append_body(f, screen)

class packet_type(Enum):
  SM2_CTRL_FW = 0x0001
  A400_CTRL_FW = 0x0002
  J1_CTRL_FW = 0x0003
  SM2_MODULE_FW = 0x0004
  ESP32_MODULE_FW = 0x0005

class ugr_ctrl_flag(Enum):
  UGR_NORMAL = 0x00
  UGR_FORCE = 0x01

class ugr_status(Enum):
  UGR_STATUS_FIRST_BURN = 0xAA00
  UGR_STATUS_FIRST_WAIT = 0xAA01
  UGR_STATUS_FIRST_START = 0xAA02
  UGR_STATUS_FIRST_TRANS = 0xAA03
  UGR_STATUS_FIRST_END = 0xAA04
  UGR_STATUS_FIRST_JUMP_APP = 0xAA05

class packet:
  def __init__(self, bin, p_type, ctrl_flag, ver, radr, s_id, e_id):
    self.magic_string = "snapmaker update.bin"
    self.protocol_ver = 0x01
    self.pack_type = p_type
    self.ugr_ctrl_flag = ctrl_flag
    self.start_index = s_id
    self.end_index = e_id
    self.fw_version = ver
    self.timestamp = "2022.04.28:18:03:01"
    self.ugr_status = 0xAA00
    self.fw_lenght = len(bin)
    self.fw_checksum = crc32(bin)
    print("%x" % self.fw_checksum)
    self.fw_runaddr = radr
    self.channel = 0
    self.peer = 0
    self.packet_checksum = 0
    self.bin = bin

  def gen(self):
    payload = bytearray(0)
    payload.extend(bytes(self.magic_string, encoding="utf-8"))
    payload.append(0x00)
    payload.append(self.protocol_ver & 0xFF)
    payload.extend(self.pack_type.to_bytes(2, 'little'))
    payload.append(self.ugr_ctrl_flag & 0xFF)
    payload.extend(self.start_index.to_bytes(2, 'little'))
    payload.extend(self.end_index.to_bytes(2, 'little'))

    b = bytearray(self.fw_version, encoding="utf-8")
    print(b)
    l = len(b)
    for i in range(l, 32):
      b.append(0)
    payload.extend(b)
    print(b)

    b = bytearray(self.timestamp, encoding="utf-8")
    print(b)
    l = len(b)
    for i in range(l, 20):
      b.append(0)
    payload.extend(b)
    print(b)

    payload.extend(self.ugr_status.to_bytes(2, 'little'))
    payload.extend(self.fw_lenght.to_bytes(4, 'little'))
    if self.fw_checksum < 0:
      payload.extend(self.fw_checksum.to_bytes(4, 'little', signed=True))
    else:
      payload.extend(self.fw_checksum.to_bytes(4, 'little'))
    payload.extend(self.fw_runaddr.to_bytes(4, 'little'))
    payload.append(self.channel & 0xFF)
    payload.append(self.peer & 0xFF)
    self.packet_checksum = crc32(payload)
    payload.extend(self.packet_checksum.to_bytes(4, 'little'))

    l = len(payload)
    b = bytearray(0)
    for i in range(l, 256):
      b.append(0)
    payload.extend(b)

    return payload


VER = "V1.1.0"
FLAG = 0
RUNADDR = 0x08005C00
START_ID = 0
END_ID = 23
OUTPUT_DIR = None
PROJ_DIR = None
parser = argparse.ArgumentParser(description="gen_header")
parser.add_argument('--dir',  '-d', help='directory of project')
parser.add_argument('--file', '-f', help='bin file name')
parser.add_argument('--type', '-t', help='packet type, 1: SM2 CONTROLER, 2: A400 CONTROLER, 3: J1 CONTROLER, 4: SM2 MODUEL, 5: ESP32 MODUEL')
parser.add_argument('--ver',  '-v', help='version')
parser.add_argument('--flag', '-c', help='upgrade control flag, 0 for normal, 1 for force')
parser.add_argument('--radr', '-a', help='firmware run address')
parser.add_argument('--output', '-o', help='output directory')
args = parser.parse_args()

try:
  FILE = args.file
  TYPE = int(args.type)
  OUTPUT_DIR = args.output
  PDIR = args.dir
except:
  print("unsupported param")
  while True:
    time.sleep(1)

try:
  VER = args.ver
  FLAG = int(args.flag)
  RUNADDR = int(args.radr, 16)
except:
  pass

if VER == None and PDIR != None:
    with open(join(PDIR, 'Marlin', 'src', 'configuration.h'), 'r', encoding='utf-8') as version_file:
        lines = version_file.readlines()

    pattern = r"v\d+\.\d+\.\d+"
    for line in lines:
        match_obj = re.search(pattern, line, re.I)
        if match_obj:
            print("got version: {}".format(match_obj[0]))
            VER = match_obj[0]
            break

if VER == None:
    raise RuntimeError("No version specify!")

# _(self, bin, p_type, ctrl_flag, ver, radr):
f = open(FILE, 'rb')
bin = f.read()
print("file lenght %d" % len(bin))

pt = packet(bin, TYPE, FLAG, VER, RUNADDR, START_ID, END_ID)
head = pt.gen()

hex_str = " ".join(["{:02x}".format(x) for x in head])
print(hex_str)

date  = datetime.datetime.today().strftime('%Y%m%d')
image = "SM3_EM_APP_{}_{}.bin".format(VER, date)

if isinstance(OUTPUT_DIR, str):
  of = join(OUTPUT_DIR, image)
else:
  raise RuntimeError("must specify output dir!")

print("======== INFORMATION ======")
print("flie: " + FILE)
print("type: %d" % TYPE)
print("ver: " + VER)
print("flag: %d " % FLAG)
print("entry addr: 0x%x" % RUNADDR)
print("output file: {}".format(of))

f = open(of, 'wb')
f.write(head)
f.write(bin)
f.close()

pack_major_image(None, Path(of), None, VER)

if __name__=='__main__':
    pass