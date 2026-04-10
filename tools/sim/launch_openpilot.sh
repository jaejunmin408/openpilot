#!/usr/bin/env bash

source "$(dirname "$0")/sim_env.sh"

# Kill previous instances and clean stale IPC files
pkill -9 -f "manager.py" 2>/dev/null
pkill -9 -f "run_bridge" 2>/dev/null
sleep 1
rm -f /dev/shm/msgq_* /tmp/visionipc_* /tmp/stats 2>/dev/null
rm -f ~/.comma/params/d/CarParams* 2>/dev/null

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export SKIP_FW_QUERY="1"
export FINGERPRINT="HONDA_CIVIC_2022"

export BLOCK="${BLOCK},camerad,loggerd,encoderd,micd,logmessaged,manage_athenad,soundd"
if [[ "$CI" ]]; then
  # TODO: offscreen UI should work
  export BLOCK="${BLOCK},ui"
fi

python3 -c "
from openpilot.selfdrive.test.helpers import set_params_enabled; set_params_enabled()
from openpilot.common.params import Params; Params().put_bool('AlphaLongitudinalEnabled', True)
"

cd $OPENPILOT_DIR/system/manager && exec ./manager.py
