#!/usr/bin/env bash
# Install / remove the rm-drone systemd units. Run with sudo.
#   sudo scripts/install_systemd.sh [--user NAME] [--enable]
#   sudo scripts/install_systemd.sh --uninstall
set -euo pipefail

ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
SCRIPTS="$ROOT/scripts"
UNITS="$ROOT/config/systemd"
UNIT_DIR=/etc/systemd/system
RUN_USER="${SUDO_USER:-$USER}"
ENABLE=0

while [ $# -gt 0 ]; do
  case "$1" in
    --user) RUN_USER=$2; shift ;;
    --enable) ENABLE=1 ;;
    --uninstall)
      systemctl disable --now rm-drone.target 2>/dev/null || true
      rm -f "$UNIT_DIR/rm-drone@.service" "$UNIT_DIR/rm-drone.target"
      systemctl daemon-reload
      echo "removed"; exit 0 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
  shift
done

[ "$(id -u)" -eq 0 ] || { echo "run as root (sudo)" >&2; exit 1; }

chmod +x "$SCRIPTS"/*.sh
install -d -o "$RUN_USER" /var/log/rm_drone /var/log/rm_drone/ros /etc/rm_drone
[ -f /etc/rm_drone/rm_drone.env ] || install -m 644 "$UNITS/rm_drone.env.example" /etc/rm_drone/rm_drone.env

sed -e "s|@USER@|$RUN_USER|" -e "s|@SCRIPTS@|$SCRIPTS|" \
  "$UNITS/rm-drone@.service" > "$UNIT_DIR/rm-drone@.service"
install -m 644 "$UNITS/rm-drone.target" "$UNIT_DIR/rm-drone.target"
systemctl daemon-reload

if [ "$ENABLE" -eq 1 ]; then
  systemctl enable rm-drone.target
  echo "enabled; starts on boot"
else
  echo "installed; start with: systemctl start rm-drone.target"
fi
