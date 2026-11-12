#!/usr/bin/env bash
# Linux native gcc only. No flashing, MCU reset, package installation or downloads.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build="$root/build/health_host_tests"
mkdir -p -- "$build"
cc="${CC:-gcc}"
"$cc" -std=c11 -O2 -g -Wall -Wextra -Werror -Wpedantic \
  -ffunction-sections -fdata-sections \
  -I"$root/tests/stubs" -I"$root/Core/Inc" \
  "$root/tests/test_health_state.c" -Wl,--gc-sections \
  -o "$build/test_health_state"
"$build/test_health_state"
# Verify the reset path was discarded rather than mocked into a no-op executable.
if nm "$build/test_health_state" | awk '{ print $3 }' | \
    grep -Eq '^(robot_health_reset_with_reason|NVIC_SystemReset|health_host_bkp)$'; then
  printf 'FAIL: reset-only symbols unexpectedly retained\n' >&2
  exit 1
fi
printf 'PASS linker discarded uncalled reset path\n'
