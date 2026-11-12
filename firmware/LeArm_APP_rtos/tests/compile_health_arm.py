"""Compile changed Cortex-M3 units; no firmware linking, flashing or device access."""
from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parents[1]
output = root / 'build/health_arm_objects'
output.mkdir(parents=True, exist_ok=True)
compiler = shutil.which('arm-none-eabi-gcc')
if not compiler:
    raise SystemExit('Put the existing STM32 ARM GCC toolchain on PATH first')
includes = [
    'Core/Inc', 'Drivers/STM32F1xx_HAL_Driver/Inc', 'Drivers/STM32F1xx_HAL_Driver/Inc/Legacy',
    'Drivers/CMSIS/Device/ST/STM32F1xx/Include', 'Drivers/CMSIS/Include',
    'Hiwonder', 'Hiwonder/Peripherals/Inc', 'Hiwonder/Portings/Inc', 'Hiwonder/Arm/Inc',
    'Hiwonder/Middlewares/Inc', 'Hiwonder/Chassis/Inc', 'Hiwonder/Third_Party/U8g2',
    'Middlewares/Third_Party/FreeRTOS/Source/include',
    'Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2',
    'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3',
]
base = [compiler, '-mcpu=cortex-m3', '-mthumb', '-std=c11', '-Wall', '-Wextra',
        '-DUSE_HAL_DRIVER', '-DSTM32F103xB', '-ffunction-sections', '-fdata-sections', '-fstack-usage']
for folder in includes:
    base += ['-I', str(root / folder)]
files = ['RobotHealthWire.c', 'RobotHealth.c', 'ChassisTask.c', 'ArmTask.c',
         'HealthTask.c', 'RobotSafety.c', 'SerialTask.c', 'RobotProtocol.c']
for name in files:
    subprocess.run(base + ['-c', str(root / 'Core/Src' / name), '-o', str(output / (name + '.o'))], check=True)
    print('PASS ARM compile:', name, flush=True)
print('PASS: 8 translation units; stack-usage reports under build/health_arm_objects.')
print('This is not a linked or hardware-validated firmware image.')
