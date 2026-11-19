#!/usr/bin/env bash
# Build + run the core tests with the host compiler (same files `pio test -e native` uses).
# Fallback for machines where the PlatformIO registry is unreachable.
set -euo pipefail
cd "$(dirname "$0")/.."
UNITY_DIR=${UNITY_DIR:-.unity}
if [ ! -f "$UNITY_DIR/src/unity.c" ]; then
  git clone -q --depth 1 --branch v2.6.0 https://github.com/ThrowTheSwitch/Unity "$UNITY_DIR"
fi
mkdir -p .build
gcc -O1 -c "$UNITY_DIR/src/unity.c" -I "$UNITY_DIR/src" -o .build/unity.o
g++ -std=c++17 -Wall -Wextra -Werror -O1 -I lib/gantry_core/src -I "$UNITY_DIR/src" \
  lib/gantry_core/src/gantry_core.cpp test/test_core/test_main.cpp .build/unity.o -o .build/test_core
./.build/test_core
