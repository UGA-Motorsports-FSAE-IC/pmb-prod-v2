#!/usr/bin/env bash
# Self-checks that need no hardware and no ARM toolchain.
#
#   test_pid.c      the firmware's fixed-point control math (needs a host cc)
#   test_pmbtune.py the identification and gain-derivation math (needs numpy)
#
# Both are regression tests for specific defects. If either fails, the gains
# the tuning suite prints are not trustworthy.
#
# Windows: run this from Git Bash or WSL. If you would rather not, TUNING.md §2
# lists the two commands this script runs, to paste straight into PowerShell.
set -uo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT

c_result="skipped"
py_result="skipped"

echo "== firmware control math (test_pid.c) =="
cc_bin=""
for candidate in "${CC:-}" gcc cc clang; do
    [ -n "$candidate" ] && command -v "$candidate" >/dev/null 2>&1 && { cc_bin="$candidate"; break; }
done

if [ -z "$cc_bin" ]; then
    echo "SKIPPED: no host C compiler found (tried \$CC, gcc, cc, clang)."
    echo "         Linux: install gcc.  Windows: install MSYS2/MinGW gcc, or"
    echo "         run the two commands in TUNING.md Section 2 by hand."
else
    if "$cc_bin" -std=c11 -Wall -Wextra -Werror \
           -I "$root/Core/Inc" \
           "$root/tests/test_pid.c" "$root/Core/Src/pid.c" \
           -o "$out/test_pid" && "$out/test_pid"; then
        c_result="passed"
    else
        c_result="FAILED"
    fi
fi

echo
echo "== tuning suite math (test_pmbtune.py) =="
py_bin=""
for candidate in python3 python py; do
    command -v "$candidate" >/dev/null 2>&1 && { py_bin="$candidate"; break; }
done

if [ -z "$py_bin" ]; then
    echo "SKIPPED: no python interpreter found"
elif ! "$py_bin" -c "import numpy" >/dev/null 2>&1; then
    echo "SKIPPED: numpy not installed (pip install -r tools/requirements.txt)"
else
    if "$py_bin" "$root/tests/test_pmbtune.py" | tail -1; then
        py_result="passed"
    else
        py_result="FAILED"
    fi
fi

echo
echo "== summary =="
echo "  control math (C)   : $c_result"
echo "  tuning math (numpy): $py_result"

# A skip is reported loudly but is not a failure; a real failure is.
case "$c_result$py_result" in
    *FAILED*) exit 1 ;;
esac
