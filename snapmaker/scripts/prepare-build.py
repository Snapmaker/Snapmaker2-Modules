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
import shutil
import sys
from os.path import isdir, isfile, join

Import("env")

platform = env.PioPlatform()
board = env.BoardConfig()

print ("++++++++++++++++++++++++++++++Prepare env start++++++++++++++++++++++++++++++" )
print ("Prepare env for " + env.get("BOARD"))

frwk = env.get("PIOFRAMEWORK")
pkg = platform.frameworks[frwk[0]]["package"]

pkg_dir = platform.get_package_dir(pkg)

print ("Package is " + pkg)
print ("Location: " + pkg_dir)

mcu = board.get('build.mcu')

build_script = "platformio-build-%s.py" % mcu[0:7]
build_script_path = join(pkg_dir, "tools", build_script)
shutil.copy(join(sys.path[0], build_script), build_script_path)

"""
frwk_dir = join(pkg_dir, mcu[0:6].upper())
if isdir(frwk_dir):
  print "%s is exist." % frwk_dir
  # TODO: here need to add comparation between src and dst framework
else:
  print "%s is not exist." % frwk_dir
  print "now create it"
  shutil.copytree(join(sys.path[0], mcu[0:6].upper()), frwk_dir)
"""

print ("++++++++++++++++++++++++++++++Prepare env end++++++++++++++++++++++++++++++" )