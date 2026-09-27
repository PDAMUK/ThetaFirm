#!/bin/bash
# Prepare a local environment for the batch-mode simulation tests:
#   * clone Kalico (at KALICO_REF) into $KALICO_DIR
#   * build the stm32f407 MCU data dictionary (needs arm-none-eabi-gcc)
#   * create a Python virtualenv with Kalico's dependencies and pytest
#
# Usage:  kalico/tests/setup_env.sh [workdir]
# Then run the printed "export" lines and:  cd kalico/tests && pytest
set -e

WORKDIR="$(realpath -m "${1:-$HOME/.cache/thetafirm-sim}")"
KALICO_REF="${KALICO_REF:-84a4105726e22c5c15943e791b5cdb3beff52e77}"
KALICO_DIR="${KALICO_DIR:-$WORKDIR/kalico}"
mkdir -p "$WORKDIR"

if [ ! -d "$KALICO_DIR/.git" ]; then
    git clone https://github.com/KalicoCrew/kalico.git "$KALICO_DIR"
fi
git -C "$KALICO_DIR" fetch --quiet origin
git -C "$KALICO_DIR" checkout --quiet "$KALICO_REF"

if ! command -v arm-none-eabi-gcc >/dev/null; then
    echo "arm-none-eabi-gcc not found (apt install gcc-arm-none-eabi" \
         "libnewlib-arm-none-eabi)" >&2
    exit 1
fi
(
    cd "$KALICO_DIR"
    cp test/configs/stm32f407.config .config
    make olddefconfig >/dev/null
    make -j"$(nproc)" >/dev/null
)
cp "$KALICO_DIR/out/klipper.dict" "$WORKDIR/stm32f407.dict"

if [ ! -x "$WORKDIR/venv/bin/python" ]; then
    python3 -m venv "$WORKDIR/venv"
fi
"$WORKDIR/venv/bin/pip" install --quiet "cffi>=1.15.1" "greenlet>=2.0.2" \
    Jinja2 "markupsafe>=2.1.5" numpy "pyserial>=3.4" "python-can>=4.6" pytest

echo "export KALICO_DIR=$KALICO_DIR"
echo "export KALICO_DICT=$WORKDIR/stm32f407.dict"
echo "export KALICO_PY=$WORKDIR/venv/bin/python"
echo "export PATH=$WORKDIR/venv/bin:\$PATH"
