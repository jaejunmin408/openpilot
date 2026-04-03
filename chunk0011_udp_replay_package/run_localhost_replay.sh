#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python "${SCRIPT_DIR}/tools/udp_replay_player.py" \
  --plan-bank "${SCRIPT_DIR}/chunk0011_model_plan_bank.npz" \
  --host "127.0.0.1" \
  --port 5001 \
  --control-dt 0.02 \
  --control-points 25 \
  --coord-mode local \
  "$@"
