#!/usr/bin/env bash
source "$(dirname "$0")/sim_env.sh"
exec python3 "$OPENPILOT_DIR/tools/sim/test_udp_sender.py" "$@"
