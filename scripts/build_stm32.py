#!/usr/bin/env python3
"""Build the maintained LeArm Keil source list with ARM GCC, without flashing."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import xml.etree.ElementTree as ET

from check_stm32_project import DEFAULT_SOURCE, validate_project


def run(args):
    subprocess.run([str(a) for a in args], check=True)


def cmake_path(path):
    return '"' + str(path).replace('\\', '/').replace('"', '\\"') + '"'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default=DEFAULT_SOURCE, type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'build/stm32')
    parser.add_argument('--toolchain-bin', type=Path)
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    try:
        validate_project(source)
    except (OSError, ValueError, ET.ParseError) as error:
        parser.error(str(error))
    compiler = shutil.which('arm-none-eabi-gcc') if args.toolchain_bin is None else str(args.toolchain_bin.resolve() / 'arm-none-eabi-gcc')
    if not compiler or not Path(compiler).is_file():
        parser.error('install gcc-arm-none-eabi or supply --toolchain-bin')
    compiler = Path(compiler)
    project = source / 'MDK-ARM/LeArm.uvprojx'
    target = ET.parse(project).find('./Targets/Target')
    if target is None:
        parser.error('Keil target not found')

    def resolve(name):
        return (project.parent / name.replace('\\', '/')).resolve()

    sources, libraries = [], []
    for group in target.findall('./Groups/Group'):
        if group.findtext('./GroupOption/CommonProperty/IncludeInBuild') == '0':
            continue
        for item in group.findall('./Files/File'):
            if item.findtext('./FileOption/CommonProperty/IncludeInBuild') == '0':
                continue
            path = resolve(item.findtext('FilePath'))
            if path.suffix.lower() == '.c':
                sources.append(path)
            elif path.suffix.lower() in ('.lib', '.a'):
                libraries.append(path)
    # ARMASM startup in Keil is replaced by the existing GNU startup, not translated.
    sources += [source / 'startup_stm32f103xb.s', source / 'Core/Src/sysmem.c']
    sources = list(dict.fromkeys(sources))
    includes = [resolve(s) for s in target.findtext('.//Cads/VariousControls/IncludePath').split(';') if s]
    defines = [s for s in target.findtext('.//Cads/VariousControls/Define').split(',') if s]
    for path in sources + includes + libraries:
        if not path.exists():
            parser.error(f'missing Keil build input: {path}')
    scatter = source / 'MDK-ARM/LeArm/LeArm.sct'
    match = re.search(r'LR_IROM1\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)', scatter.read_text())
    if not match or tuple(int(v, 16) for v in match.groups()) != (0x08004000, 0x1BC00):
        parser.error('unrecognized IAP app layout; expected 0x08004000 + 0x1BC00')
    output.mkdir(parents=True, exist_ok=True)
    generated = output / 'generated'
    generated.mkdir(exist_ok=True)
    # ArmClang accepts a static const integer as an array bound; ISO C/GCC
    # requires an integer constant expression. Build a copy with equal value.
    ultrasound = source / 'Hiwonder/Peripherals/Src/ultrasound.c'
    ultrasound_bytes = ultrasound.read_bytes()
    if b'const static uint8_t filter_num = 3;' in ultrasound_bytes:
        portable = generated / 'ultrasound.c'
        portable.write_bytes(ultrasound_bytes.replace(b'const static uint8_t filter_num = 3;',
                                                     b'enum { filter_num = 3 };'))
        sources[sources.index(ultrasound)] = portable
    # Windows file names are case insensitive; retain the actual Linux header.
    (generated / 'Kinematics.h').write_text('#include "kinematics.h"\n')
    includes.insert(0, generated)
    runtime = generated / 'runtime.c'
    runtime.write_text('/* GNU startup calls __libc_init_array; no board init hooks. */\nvoid _init(void) {}\nvoid _fini(void) {}\n')
    sources.append(runtime)
    # Keep ST's original license and startup symbols; reserve Boot + metadata.
    linker = (source / 'STM32F103XX_FLASH.ld').read_text()
    linker, count = re.subn(r'FLASH \(rx\)\s*: ORIGIN = 0x8000000, LENGTH = 128K',
                           'FLASH (rx) : ORIGIN = 0x08004000, LENGTH = 0x1BC00', linker)
    if count != 1:
        parser.error('unexpected CubeMX linker script')
    linker = linker.replace('(READONLY)', '')
    linker = re.sub(r'/DISCARD/\s*:\s*\{.*?\}', '', linker, flags=re.S)
    (generated / 'app.ld').write_text(linker)
    cmake = '''cmake_minimum_required(VERSION 3.22)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
'''
    cmake += f'set(CMAKE_C_COMPILER {cmake_path(compiler)})\nset(CMAKE_ASM_COMPILER {cmake_path(compiler)})\n'
    cmake += 'project(LeArmLinux C ASM)\nset(CMAKE_EXPORT_COMPILE_COMMANDS ON)\n'
    cmake += 'add_executable(LeArm.elf\n' + '\n'.join(cmake_path(p) for p in sources) + ')\n'
    cmake += 'target_include_directories(LeArm.elf PRIVATE\n' + '\n'.join(cmake_path(p) for p in includes) + ')\n'
    cmake += 'target_compile_definitions(LeArm.elf PRIVATE ' + ' '.join(defines) + ')\n'
    cmake += '''target_compile_options(LeArm.elf PRIVATE -mcpu=cortex-m3 -mthumb -Os -g3
 -ffunction-sections -fdata-sections -fstack-usage -Wall -Wextra
 $<$<COMPILE_LANGUAGE:C>:-std=gnu11>)
target_link_options(LeArm.elf PRIVATE -mcpu=cortex-m3 -mthumb -nostartfiles
 --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage
 -T${CMAKE_CURRENT_SOURCE_DIR}/app.ld -Wl,-Map=${CMAKE_BINARY_DIR}/LeArm.map)
target_link_libraries(LeArm.elf PRIVATE m)
'''
    cmake += 'target_link_libraries(LeArm.elf PRIVATE ' + ' '.join(cmake_path(p) for p in libraries) + ' m)\n'
    # Debian packages extracted in a user directory need an explicit nano include.
    newlib = compiler.parent.parent / 'include/newlib'
    if newlib.exists():
        cmake += f'target_include_directories(LeArm.elf SYSTEM BEFORE PRIVATE {cmake_path(newlib / "nano")} {cmake_path(newlib)})\n'
    (generated / 'CMakeLists.txt').write_text(cmake)
    build = output / 'cmake'
    run(['cmake', '-S', generated, '-B', build, '-G', 'Ninja'])
    run(['cmake', '--build', build, '--parallel', min(os.cpu_count() or 2, 8)])
    prefix = compiler.parent / 'arm-none-eabi-'
    elf, binary, hexfile = build / 'LeArm.elf', output / 'LeArm.bin', output / 'LeArm.hex'
    startup = subprocess.check_output([str(prefix) + 'objdump', '-d', '--disassemble=main', elf], text=True)
    calls = [startup.find(f'<{name}>') for name in
             ('robot_health_init', 'osKernelInitialize', 'MX_FREERTOS_Init', 'osKernelStart')]
    if any(position < 0 for position in calls) or calls != sorted(calls):
        raise RuntimeError('linked main must initialize health and RTOS objects before starting the kernel')
    setup = subprocess.check_output([str(prefix) + 'objdump', '-d',
                                     '--disassemble=MX_FREERTOS_Init', elf], text=True)
    safety_call, worker_call = setup.find('<robot_safety_init>'), setup.find('<osThreadNew>')
    if not (0 <= safety_call < worker_call):
        raise RuntimeError('linked RTOS startup must initialize safety before creating workers')
    print('PASS linked startup: health -> kernel initialize -> safety -> workers -> kernel start')
    run([str(prefix) + 'objcopy', '-O', 'binary', elf, binary])
    run([str(prefix) + 'objcopy', '-O', 'ihex', elf, hexfile])
    image = binary.read_bytes()
    sp, reset = struct.unpack_from('<II', image)
    if not (8 <= len(image) <= 0x1BC00 and 0x20000000 < sp <= 0x20005000 and sp % 8 == 0
            and reset & 1 and 0x08004000 <= (reset & ~1) < 0x08004000 + len(image)):
        raise RuntimeError('invalid firmware size/vector table')
    inputs = [p for p in sources if p.is_relative_to(source)] + [ultrasound]
    inputs += [p for p in source.rglob('*.h') if 'build' not in p.relative_to(source).parts]
    inputs += libraries + [project, scatter, source / 'STM32F103XX_FLASH.ld']
    manifest = {'compiler': subprocess.check_output([compiler, '--version'], text=True).splitlines()[0],
                'app_start': '0x08004000', 'app_limit': '0x0801FC00', 'size': len(image),
                'initial_sp': f'0x{sp:08X}', 'reset_handler': f'0x{reset:08X}',
                'sha256': hashlib.sha256(image).hexdigest(), 'source_count': len(sources),
                'portable_ultrasound_sha256': hashlib.sha256(ultrasound_bytes.replace(b'const static uint8_t filter_num = 3;', b'enum { filter_num = 3 };')).hexdigest(),
                'inputs_sha256': {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(inputs))}}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    run([str(prefix) + 'size', elf])
    print(json.dumps({k: v for k, v in manifest.items() if k != 'inputs_sha256'}, indent=2))
    print(f'Firmware: {binary}\nNot flashed; hardware behavior still needs validation.')


if __name__ == '__main__':
    main()
