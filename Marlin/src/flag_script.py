#
# Snapmaker2-Modules Firmware
# Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
#
# This file is part of Snapmaker2-Modules
# (see https://github.com/Snapmaker/Snapmaker2-Modules)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
from __future__ import print_function
import sys

#dynamic build flags for generic compile options
if __name__ == "__main__":
  args = " ".join([ "-std=gnu11",
                    "-std=gnu++11",
                    "-Os",
                    "-mcpu=cortex-m3",
                    "-mthumb",

                    "-ffreestanding",
                    "-fsigned-char",
                    "-fno-move-loop-invariants",
                    "-fno-strict-aliasing",
                    # "-fexceptions",

                    "--specs=nano.specs",
                    "--specs=nosys.specs",

                    "-MMD",
                    "-MP",
                  ])

  for i in range(1, len(sys.argv)):
    args += " " + sys.argv[i]

  print(args)

# extra script for linker options
else:
  from SCons.Script import DefaultEnvironment
  env = DefaultEnvironment()
  env.Append(
      ARFLAGS=["rcs"],

      ASFLAGS=["-x", "assembler-with-cpp"],

      CXXFLAGS=[
          "-fabi-version=0",
          "-fno-use-cxa-atexit",
          "-fno-threadsafe-statics"
      ],
      LINKFLAGS=[
          "-Os",
          "-mcpu=cortex-m3",
          "-ffreestanding",
          "-mthumb",
          "--specs=nano.specs",
          "--specs=nosys.specs",
          "-u_printf_float",
      ],
  )
