#!/bin/bash
# Install the ThetaFirm plugin into a Kalico installation.
#
#   ./install.sh [path to Kalico]      (default: ~/klipper, then ~/kalico)
#
# The plugin is symlinked into Kalico's klippy/plugins directory, which
# Kalico ignores in git, so Kalico updates keep working.  Re-running the
# script is safe.  Restart Kalico afterwards (sudo systemctl restart
# klipper).
set -e

SRCDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLUGINS="core_rtheta.py"

find_kalico() {
    for d in "$@"; do
        if [ -d "$d/klippy" ]; then
            echo "$d"
            return
        fi
    done
}

KALICO_DIR="$(find_kalico "${1:-}" "$HOME/klipper" "$HOME/kalico")"
if [ -z "$KALICO_DIR" ]; then
    echo "Kalico installation not found; pass its path as an argument" >&2
    exit 1
fi
if [ ! -d "$KALICO_DIR/klippy/plugins" ]; then
    echo "$KALICO_DIR has no klippy/plugins directory." >&2
    echo "ThetaFirm needs Kalico (https://github.com/KalicoCrew/kalico)," \
         "not mainline Klipper." >&2
    exit 1
fi

for p in $PLUGINS; do
    ln -sfn "$SRCDIR/plugins/$p" "$KALICO_DIR/klippy/plugins/$p"
    echo "Linked $KALICO_DIR/klippy/plugins/$p"
done

cat <<EOF

Next steps:
  1. Copy (or [include]) $SRCDIR/config/printer.cfg and macros.cfg into
     your printer configuration directory and set the [mcu] serial.
  2. Restart Kalico:  sudo systemctl restart klipper
  3. Follow the commissioning checklist in $SRCDIR/README.md before the
     first homing move.

Optional Moonraker update manager entry (moonraker.conf):

[update_manager thetafirm]
type: git_repo
path: $(dirname "$SRCDIR")
origin: https://github.com/pdamuk/ThetaFirm.git
primary_branch: main
install_script: kalico/install.sh
managed_services: klipper
EOF
