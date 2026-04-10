#!/usr/bin/env bash
# Common environment setup for simulation scripts

# Display: use real display :0 if available, fallback to Xvfb
if [ -z "$DISPLAY" ]; then
  if [ -e /tmp/.X11-unix/X0 ]; then
    export DISPLAY=:0
  else
    export DISPLAY=:99
    if ! pgrep -x Xvfb > /dev/null; then
      Xvfb :99 -screen 0 1920x1080x24 &
      sleep 1
    fi
  fi
fi

# venv
SCRIPT_DIR=$(dirname "${BASH_SOURCE[0]}")
OPENPILOT_DIR=$(cd "$SCRIPT_DIR/../../" && pwd)
if [ -z "$VIRTUAL_ENV" ] && [ -f "$OPENPILOT_DIR/.venv/bin/activate" ]; then
  source "$OPENPILOT_DIR/.venv/bin/activate"
fi

export OPENPILOT_DIR
