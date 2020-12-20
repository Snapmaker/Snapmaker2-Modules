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
from os.path import join,isfile,isdir
from platform import system

Import("env")

board = env.BoardConfig()

print ("++++++++++++++++++++++++++++++prepare upload start++++++++++++++++++++++++++++++\n")

upload_protocol = env.subst("$UPLOAD_PROTOCOL")


if upload_protocol.startswith("jlink"):
    def _jlink_cmd_script(env, source):
        build_dir = env.subst("$BUILD_DIR")
        if not isdir(build_dir):
            makedirs(build_dir)
        script_path = join(build_dir, "upload-snap.jlink")
        commands = [
            "h",
            "loadbin %s, %s" % (source, board.get(
                "build.vect_table_addr", "0x08000000")),
            "r",
            "q"
        ]
        with open(script_path, "w") as fp:
            fp.write("\n".join(commands))
        return script_path
    env.Replace(
        __jlink_cmd_script=_jlink_cmd_script,
        UPLOADER="JLink.exe" if system() == "Windows" else "JLinkExe",
        UPLOADERFLAGS=[
            "-device", board.get("debug", {}).get("jlink_device"),
            "-speed", "4000",
            "-if", ("jtag" if upload_protocol == "jlink-jtag" else "swd"),
            "-autoconnect", "1"
        ],
        UPLOADCMD='$UPLOADER $UPLOADERFLAGS -CommanderScript "${__jlink_cmd_script(__env__, SOURCE)}"'
    )
elif upload_protocol.startswith("stlink"):
    print ("st is not supprted now!")

