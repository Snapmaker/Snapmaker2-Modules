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
import sys
from datetime import datetime
from shutil import move
from os.path import join,isfile,getsize
Import("env", "projenv")


print ("++++++++++++++++++++++++++++++CPPDEFINES start++++++++++++++++++++++++++++++")
print (projenv.get("CPPDEFINES", [])[:])
print ("++++++++++++++++++++++++++++++CPPDEFINES end++++++++++++++++++++++++++++++\n")

print ("++++++++++++++++++++++++++++++CPPPATH start++++++++++++++++++++++++++++++")
cpp_path = projenv.get("CPPPATH", [])[:]
for p in cpp_path:
  print (p)
print ("++++++++++++++++++++++++++++++CPPPATH end++++++++++++++++++++++++++++++\n" )

print ("++++++++++++++++++++++++++++++CCFLAGS start++++++++++++++++++++++++++++++")
print (projenv.get("CCFLAGS", [])[:])
print ("++++++++++++++++++++++++++++++CCFLAGS end++++++++++++++++++++++++++++++\n")

print ("++++++++++++++++++++++++++++++CXXFLAGS start++++++++++++++++++++++++++++++")
print (projenv.get("CXXFLAGS", [])[:])
print ("++++++++++++++++++++++++++++++CXXFLAGS end++++++++++++++++++++++++++++++\n")

print ("++++++++++++++++++++++++++++++LINKFLAGS start++++++++++++++++++++++++++++++")
print (projenv.get("LINKFLAGS", [])[:])
print ("LDSCRIPT_PATH: " + projenv.get("LDSCRIPT_PATH"))
print ("++++++++++++++++++++++++++++++LINKFLAGS end++++++++++++++++++++++++++++++\n")

print ("++++++++++++++++++++++++++++++LIBPATH start++++++++++++++++++++++++++++++")
lib_path = projenv.get("LIBPATH", [])[:]
for p in lib_path:
  print (p)
print ("++++++++++++++++++++++++++++++LIBPATH end++++++++++++++++++++++++++++++\n" )
print ("++++++++++++++++++++++++++++++LIBSOURCE_DIRS start++++++++++++++++++++++++++++++")
lib_src = projenv.get("LIBSOURCE_DIRS", [])[:]
for p in lib_src:
  print (p)
print ("++++++++++++++++++++++++++++++LIBSOURCE_DIRS end++++++++++++++++++++++++++++++\n" )


print ("++++++++++++++++++++++++++++++record env start++++++++++++++++++++++++++++++")
def save_env():
  cwd = sys.path[0]
  print ("current dir: " + cwd)

  # get file path and see if it is too large
  f_path = join(cwd, "env.txt")
  if isfile(f_path):
    # about 85 kB / time, we git it 500kB
    # if it is over 500kB, jsut rename to env_old.txt
    if getsize(f_path) > 500 * 1024:
      move(f_path, join(cwd, "env_old.txt"))

  f = open(f_path, "a+")

  # build timestamp
  f.write("==============================\r\n")
  f.write(datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
  f.write("\r\n")

  # project env, we need to focus on it
  f.write("+++++++++++++++projenv start+++++++++++++++\r\n")
  f.write(str(projenv.Dump()))
  f.write("\r\n+++++++++++++++projenv end+++++++++++++++\r\n")

  # globle env
  f.write("\r\n+++++++++++++++env start+++++++++++++++\r\n")
  f.write(str(env.Dump()))
  f.write("\r\n+++++++++++++++env end+++++++++++++++\r\n")

  f.close()
save_env()
print ("++++++++++++++++++++++++++++++record env end++++++++++++++++++++++++++++++\n")