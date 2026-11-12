#!/usr/bin/env bash
# Host-only tests of real firmware C files; never opens hardware.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build="$root/build/safety_host_tests"
mkdir -p -- "$build"
cc="${CC:-gcc}"
"$cc" -std=c11 -O2 -g -Wall -Wextra -Werror -Wpedantic \
  -I"$root/tests/stubs" -I"$root/Core/Inc" \
  "$root/tests/test_safety_startup.c" "$root/Core/Src/SafetyManager.c" \
  -o "$build/test_safety_startup"
"$build/test_safety_startup"
