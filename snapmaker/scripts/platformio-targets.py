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
from os.path import join

Import("env", "projenv")

# Single action/command per 1 target
# env.AddCustomTarget("sysenv", None, 'python -c "import os; print(os.environ)"'))


# Multiple actions
# env.AddCustomTarget(
#     name="pioenv",
#     dependencies=None,
#     actions=[
#     "pio --version",
#     "python --version"
#     ],
#     title="Core Env",
#     description="Show PlatformIO Core and Python versions"
# )


project_dir = projenv.get("PROJECT_DIR")

pack_script = join(project_dir, 'snapmaker', 'scripts', 'pack.py')

fw_bin = join(projenv.get("PROJECT_BUILD_DIR"), projenv.get("PIOENV"), projenv.get("PROGNAME") + '.bin')


env.AddCustomTarget(
    name="pack",
    dependencies=None,
    actions=[
    "python {0} -d {1} -m {2} ".format(pack_script, project_dir, fw_bin),
    ],
    title="Pack",
    description="Pack Snapmaker Firmware"
)
