#!/usr/bin/env bash
# Self-checks that need no hardware and no ARM toolchain.
#
#   test_pid.c      the firmware's fixed-point control math (native gcc)
#   test_pmbtune.py the identification and gain-derivation math (numpy)
#
# Both are regression tests for specific defects. If either fails, the gains
# the tuning suite prints are not trustworthy.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT

echo "== firmware control math (test_pid.c) =="
gcc -std=c11 -Wall -Wextra -Werror \
    -I "$root/Core/Inc" \
    "$root/tests/test_pid.c" "$root/Core/Src/pid.c" \
    -o "$out/test_pid"
"$out/test_pid"

echo
echo "== tuning suite math (test_pmbtune.py) =="
if python3 -c "import numpy" 2>/dev/null; then
    python3 "$root/tests/test_pmbtune.py" | tail -1
else
    echo "SKIPPED: numpy not installed (pip install -r tools/requirements.txt)"
fi
