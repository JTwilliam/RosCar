#!/usr/bin/env python3
"""Check the maintained Keil App paths, partition and startup ordering."""
import argparse
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

DEFAULT_SOURCE = Path(__file__).resolve().parents[1] / 'firmware/LeArm_APP_rtos'


def c_function(source, name):
    text = re.sub(r'/\*.*?\*/|//[^\n]*', '', source, flags=re.S)
    match = re.search(r'\b' + re.escape(name) + r'\s*\([^;{}]*\)\s*\{', text)
    if match is None:
        raise ValueError(f'missing function: {name}')
    start, depth = match.end(), 1
    for end in range(start, len(text)):
        depth += (text[end] == '{') - (text[end] == '}')
        if depth == 0:
            return text[start:end]
    raise ValueError(f'unclosed function: {name}')


def require_call_order(body, names):
    positions = []
    for name in names:
        matches = list(re.finditer(r'\b' + re.escape(name) + r'\s*\(', body))
        if not matches:
            raise ValueError(f'missing startup call: {name}')
        positions.append(matches[0].start())
    if positions != sorted(set(positions)):
        raise ValueError('invalid startup order: ' + ' -> '.join(names))


def validate_project(source):
    source = Path(source).resolve()
    project = source / 'MDK-ARM/LeArm.uvprojx'
    tree = ET.parse(project)
    target = tree.find('./Targets/Target')
    if target is None:
        raise ValueError('Keil target not found')

    def resolve(name, directory=False):
        name = name.replace('\\', '/')
        if Path(name).is_absolute() or re.match(r'^[A-Za-z]:', name):
            raise ValueError(f'absolute Keil input is not portable: {name}')
        path = (project.parent / name).resolve()
        if not path.is_relative_to(source):
            raise ValueError(f'Keil input leaves App directory: {name}')
        if not (path.is_dir() if directory else path.is_file()):
            raise ValueError(f'missing Keil input: {name}')
        return path

    inputs = [resolve(item.text) for item in target.findall('./Groups/Group/Files/File/FilePath')]
    includes = target.findtext('.//Cads/VariousControls/IncludePath', '').split(';')
    for name in filter(None, includes):
        resolve(name, directory=True)
    for item in tree.findall('./RTE/files/file/instance'):
        resolve(item.text)
    scatter = resolve(target.findtext('.//LDads/ScatterFile', ''))
    match = re.search(r'LR_IROM1\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)', scatter.read_text())
    if not match or tuple(int(v, 16) for v in match.groups()) != (0x08004000, 0x1BC00):
        raise ValueError('App scatter must reserve Bootloader and metadata: 0x08004000 + 0x1BC00')
    irom = target.find('.//OnChipMemories/OCR_RVCT4')
    if irom is None or (int(irom.findtext('StartAddress'), 0), int(irom.findtext('Size'), 0)) != (0x08004000, 0x1BC00):
        raise ValueError('Keil IROM1 must match the App scatter partition')
    if source / 'Core/Src/RobotHealthWire.c' not in inputs:
        raise ValueError('Keil source list is missing RobotHealthWire.c')

    main = c_function((source / 'Core/Src/main.c').read_text(), 'main')
    require_call_order(main, ['robot_health_init', 'osKernelInitialize',
                              'MX_FREERTOS_Init', 'osKernelStart'])
    setup = c_function((source / 'Core/Src/freertos.c').read_text(), 'MX_FREERTOS_Init')
    require_call_order(setup, ['robot_safety_init', 'osThreadNew'])
    if len(re.findall(r'\brobot_safety_init\s*\(', setup)) != 1:
        raise ValueError('safety must be initialized exactly once during startup')
    guard = r'if\s*\(\s*!robot_safety_init\s*\(\s*HAL_GetTick\s*\(\s*\)\s*\)\s*\)\s*\{\s*Error_Handler\s*\('
    if not re.search(guard, setup):
        raise ValueError('safety initialization failure must stop startup via Error_Handler')
    if not re.search(r'SCB->VTOR\s*=\s*0x08004000', main):
        raise ValueError('App VTOR must remain at 0x08004000')
    return {'project': str(project), 'project_inputs': len(inputs),
            'app_start': '0x08004000', 'app_size': '0x1BC00',
            'startup_order': 'health -> kernel initialize -> safety -> workers -> kernel start'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    args = parser.parse_args()
    try:
        result = validate_project(args.source)
    except (OSError, ValueError, ET.ParseError) as error:
        parser.exit(1, f'ERROR: {error}\n')
    print(json.dumps(result, indent=2))
    print('PASS: static Keil project check; not a Keil build or hardware test')


if __name__ == '__main__':
    main()
