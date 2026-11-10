#!/usr/bin/env bash
# Compatibility entry. Prefer cd host_ws && bash start.sh.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$root/host_ws/start.sh" "$@"
