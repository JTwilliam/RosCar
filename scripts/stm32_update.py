#!/usr/bin/env python3
"""Validate/probe/update LeArm using the Pi's existing YMODEM implementation.

No serial device is opened unless --probe or --execute is specified.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
from pathlib import Path
import secrets
import struct
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pi_ws/src/learm_bridge'))
from learm_bridge.protocol import ProtocolDecoder, crc32c, decode_chassis_state


def validate_image(image):
    if not 8 <= len(image) <= 0x1BC00:
        raise ValueError('image size is outside IAP App partition')
    sp, reset = struct.unpack_from('<II', image)
    if not (0x20000000 < sp <= 0x20005000 and sp % 8 == 0):
        raise ValueError('invalid initial stack pointer')
    if not (reset & 1 and 0x08004000 <= (reset & ~1) < 0x08004000 + len(image)):
        raise ValueError('reset handler is not inside this App image')
    return sp, reset


def encode_request(message, sequence, payload=b''):
    raw = struct.pack('<BBBBHHI', 1, message, 0, 12, sequence, len(payload), 0) + payload
    raw += struct.pack('<I', crc32c(raw))
    result = bytearray([0])
    position, code = 0, 1
    for byte in raw:
        if byte == 0:
            result[position] = code
            position = len(result)
            result.append(0)
            code = 1
        else:
            result.append(byte)
            code += 1
            if code == 255:
                result[position] = code
                position = len(result)
                result.append(0)
                code = 1
    result[position] = code
    return bytes(result) + b'\x00'


def open_port(device, baud):
    import serial
    port = serial.Serial(port=None, baudrate=baud, timeout=0.05,
                         write_timeout=5, exclusive=True)
    port.port, port.dtr, port.rts = device, False, False
    port.open()
    return port


def inspect_port(port, seconds=3.0):
    decoder = ProtocolDecoder()
    counts, state, health_length = Counter(), None, None
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        for frame in decoder.feed(port.read(port.in_waiting or 1)):
            counts[f'0x{frame.message_type:02X}'] += 1
            if frame.message_type == 0x81:
                state = decode_chassis_state(frame)
            if frame.message_type == 0x83:
                health_length = len(frame.payload)
    print(f'Frames: {dict(counts)}; health payload bytes: {health_length}')
    if state is None:
        raise RuntimeError('no valid CHASSIS_STATE; check device, baud or bootloader mode')
    print(f'Safety state: {state.safety_state}; fault flags: 0x{state.fault_flags:04X}; '
          f'motor commands: {state.motor_commands}')
    return state


def wait_ack(port, sequence, timeout):
    decoder = ProtocolDecoder()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for frame in decoder.feed(port.read(port.in_waiting or 1)):
            if frame.message_type in (0xF0, 0xF1) and len(frame.payload) == 3:
                received_sequence, code = struct.unpack('<HB', frame.payload)
                if received_sequence == sequence:
                    if frame.message_type != 0xF0 or code != 0:
                        raise RuntimeError(f'firmware rejected sequence {sequence}: NACK {code}')
                    return
    raise TimeoutError(f'no successful ACK for sequence {sequence}; update stopped')


def request_rtos_boot(port, timeout=5.0, stop_sequence=None):
    sequence = secrets.randbelow(65536) if stop_sequence is None else stop_sequence
    for message, request_sequence, payload in (
        (0x12, sequence, b''),
        (0x13, (sequence + 1) & 0xFFFF, struct.pack('<H', sequence)),
    ):
        packet = encode_request(message, request_sequence, payload)
        if port.write(packet) != len(packet):
            raise RuntimeError('short application command write')
        port.flush()
        wait_ack(port, request_sequence, timeout)
    print('STOP completed and ENTER_BOOTLOADER acknowledged')


def load_sender(path):
    spec = importlib.util.spec_from_file_location('roscar_existing_ymodem', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path)
    parser.add_argument('--port')
    parser.add_argument('--app-baud', type=int, default=115200)
    parser.add_argument('--boot-baud', type=int, default=115200)
    parser.add_argument('--sender', type=Path, default=Path.home() / 'stm32_ota/ymodem_sender.py')
    parser.add_argument('--mode', choices=('rtos', 'legacy', 'bootloader'), default='rtos')
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--probe', action='store_true', help='receive telemetry only; no commands')
    action.add_argument('--execute', action='store_true', help='stop/reset MCU and flash firmware')
    args = parser.parse_args()
    if (args.probe or args.execute) and not args.port:
        parser.error('--port is required for serial access')
    if args.probe:
        with open_port(args.port, args.app_baud) as port:
            inspect_port(port)
        return
    if args.file is None:
        parser.error('--file is required')
    image = args.file.read_bytes()
    sp, reset = validate_image(image)
    print(f'Image: {args.file}; {len(image)} bytes; SP=0x{sp:08X}; reset=0x{reset:08X}')
    print(f'SHA256: {hashlib.sha256(image).hexdigest()}')
    if not args.execute:
        print('Validation passed; serial port was not opened; firmware was not flashed')
        return
    sender = load_sender(args.sender.resolve())
    sender.validate_firmware(image)
    if args.mode == 'rtos':
        with open_port(args.port, args.app_baud) as port:
            state = inspect_port(port)
            if state.safety_state not in (1, 3) or state.fault_flags:
                raise RuntimeError('unsafe/fault state: repair hardware before software boot request')
            request_rtos_boot(port)
    elif args.mode == 'legacy':
        with open_port(args.port, args.app_baud) as port:
            sender.BootEntryRequester(port).request()
    # Use the already inspected/tested Pi sender for CRC16, retries and BL:OK.
    # Never fall back from an RTOS NACK to a legacy command or force reset.
    sender.perform_update(args.port, args.file.name, image, boot_baud=args.boot_baud,
                          serial_opener=open_port)
    print('YMODEM completed; bootloader reported BL:OK; check application telemetry next')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, TimeoutError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        sys.exit(1)
