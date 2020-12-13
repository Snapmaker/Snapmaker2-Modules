# Copyright 2014-present PlatformIO <contact@platformio.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Arduino

Arduino Wiring-based Framework allows writing cross-platform software to
control devices attached to a wide range of Arduino boards to create all
kinds of creative coding, interactive objects, spaces or physical experiences.

https://github.com/rogerclarkmelbourne/Arduino_STM32
"""

import sys
from os.path import isdir, isfile, join

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()
platform = env.PioPlatform()
board = env.BoardConfig()

# fetch vector table address from board json file
vect_table_addr = board.get("build.vect_table_addr", "0x08000000");

mcu = env.BoardConfig().get("build.mcu", "")

FRAMEWORK_ROOT = platform.get_package_dir("framework-arduinoststm32-maple")

if mcu.startswith("stm32f1"):
    FRAMEWORK_DIR = join(env.get("PROJECT_DIR"), "snapmaker", "lib", "STM32F1")
elif mcu.startswith("stm32f4"):
    FRAMEWORK_DIR = join(FRAMEWORK_ROOT, "STM32F4")
else:
    sys.stderr.write("Could not find a suitable framework variant\n")
    env.Exit(1)

assert isdir(FRAMEWORK_DIR)

# board -> variant
VARIANT_REMAP = {
    "genericSTM32F103TB": "generic_stm32f103t",
    "snapmaker_module_app": "generic_stm32f103t",
}


def get_variant(board):
    variant = VARIANT_REMAP[board] if board in VARIANT_REMAP else board
    return variant


def get_vector_tab_addr(board, upload_protocol):

    return vect_table_addr


def process_optimization_level(cpp_defines):
    if "OPTIMIZATION_FAST" in cpp_defines:
        env.Append(CCFLAGS=["-O1"])
    elif "OPTIMIZATION_FASTER" in cpp_defines:
        env.Append(CCFLAGS=["-O2"])
    elif "OPTIMIZATION_FASTEST" in cpp_defines:
        env.Append(CCFLAGS=["-O3"])
    else:
        env.Append(CCFLAGS=["-Os"])  # default optimize for size


def add_upload_protocol_defines(board, upload_protocol):
    if upload_protocol == "serial":
        env.Append(
            CPPDEFINES=[("CONFIG_MAPLE_MINI_NO_DISABLE_DEBUG", 1)])
    elif upload_protocol == "dfu":
        env.Append(CPPDEFINES=["SERIAL_USB"])
    else:
        env.Append(
            CPPDEFINES=[
                ("CONFIG_MAPLE_MINI_NO_DISABLE_DEBUG", 1),
                "SERIAL_USB"
            ])

    is_generic = board.startswith("generic") or board == "hytiny_stm32f103t"
    if upload_protocol in ("stlink", "dfu", "jlink") and is_generic:
        env.Append(CPPDEFINES=["GENERIC_BOOTLOADER"])


def get_linker_script(board, mcu, upload_protocol):

    if vect_table_addr == "0x8005C00":
        return "flash-5C00.ld"
    elif vect_table_addr == "0x8006000":
        return "flash-6000.ld"
    return "flash.ld"


def configure_error_led(board):

    if board.startswith("genericSTM32F103C"):
        led_port = "GPIOC"
        led_pin = 13
    elif board.startswith("genericSTM32F103V"):
        led_port = "GPIOE"
        led_pin = 6
    else:
        # default for all boards
        led_port = "GPIOB"
        led_pin = 1

    env.Append(
        CPPDEFINES=[
            ("ERROR_LED_PORT", led_port),
            ("ERROR_LED_PIN", led_pin)
        ]
    )

board_name = env.subst("$BOARD")
variant = get_variant(board_name)
variant_dir = join(FRAMEWORK_DIR, "variants", variant)
mcu_family = mcu[0:7].upper()
upload_protocol = env.subst("$UPLOAD_PROTOCOL")

env.Append(
    ASFLAGS=["-x", "assembler-with-cpp"],

    CFLAGS=["-std=gnu11"],

    CCFLAGS=[
        "-mcpu=%s" % env.BoardConfig().get("build.cpu"),
        "-mthumb",
        "-ffunction-sections",  # place each function in its own section
        "-fdata-sections",
        "-nostdlib",
        "-march=armv7-m",
        "--param", "max-inline-insns-single=500",
        "-Wall"
    ],

    CXXFLAGS=[
        "-std=gnu++11",
        "-fno-rtti",
        "-fno-exceptions"
    ],

    CPPDEFINES=[
        ("ARDUINO", 10808),
        "ARDUINO_ARCH_STM32",
        "ARDUINO_ARCH_%s" % mcu_family,
        "ARDUINO_%s" % (variant[0:18].upper() if board_name.startswith(
            "generic") else variant.upper()),
        "MCU_%s" % mcu[0:11].upper(),
        "__%s__" % mcu_family,
        "BOARD_%s" % variant,
        ("F_CPU", "$BOARD_F_CPU"),
        ("VECT_TAB_ADDR", get_vector_tab_addr(board_name, upload_protocol)) # inject into macro definition
    ]
)

env.Append(
    ASFLAGS=env.get("CCFLAGS", [])[:],

    CPPPATH=[
        join(FRAMEWORK_DIR, "cores", "maple"),
        join(FRAMEWORK_DIR, "system", "libmaple"),
        join(FRAMEWORK_DIR, "system", "libmaple", "include"),
        join(FRAMEWORK_DIR, "system", "libmaple", "stm32f1", "include"),
        join(FRAMEWORK_DIR, "system", "libmaple", "usb", "stm32f1"),
        join(FRAMEWORK_DIR, "system", "libmaple", "usb", "usb_lib"),
    ],

    LINKFLAGS=[
        "-Os",
        "-mthumb",
        "-mcpu=%s" % env.BoardConfig().get("build.cpu"),
        "-Wl,--check-sections",
        "-Wl,--gc-sections",
        "-Wl,--unresolved-symbols=report-all",
        "-Wl,--warn-common",
        "-Wl,--warn-section-align",
        "-Wl,--warn-unresolved-symbols"
    ],

    LIBS=["m", "gcc"],

    LIBPATH=[
        join(FRAMEWORK_DIR, "variants", variant, "ld")
    ],

    LIBSOURCE_DIRS=[
        join(FRAMEWORK_DIR, "libraries")
    ]
)

# remap ldscript
ldscript = get_linker_script(board_name, mcu, upload_protocol)
env.Replace(LDSCRIPT_PATH=ldscript)

if not isfile(join(FRAMEWORK_DIR, "variants", variant, "ld", ldscript)):
    print("Warning! Cannot find linker script for the current target!\n")

configure_error_led(board_name)
process_optimization_level(env['CPPDEFINES'])
add_upload_protocol_defines(board_name, upload_protocol)

# By default nucleo_f103rb is overclocked
if board_name == "nucleo_f103rb" and env.subst("$BOARD_F_CPU") >= 72000000:
    env.Append(CPPDEFINES=["NUCLEO_HSE_CRYSTAL"])

#
# Target: Build Core Library
#

if "build.variant" in env.BoardConfig():
    env.Append(
        CPPPATH=[variant_dir]
    )

    env.BuildSources(
        join("$BUILD_DIR", "FrameworkArduinoVariant"),
        variant_dir)


env.BuildSources(
    join("$BUILD_DIR", "FrameworkArduino"),
    join(FRAMEWORK_DIR, "cores", "maple"))
