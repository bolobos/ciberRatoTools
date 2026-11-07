#!/usr/bin/env bash
# setup.sh - agent C2_pClient (Python, no virtual environment)
#
# Executed by the bash that runs Webots, BEFORE Webots starts.
# The robot controller (C2_pClient.py) needs no compilation: only the supervisor
# (../C2_supervisor, where the Makefile is) is rebuilt with "make clean" + "make".
#
# Usage:
#   ./setup.sh             compile, then open Webots on the world $WORLD (adjust it below)
#   ./setup.sh --no-run    compile only (use this if the runner starts Webots itself,
#                          otherwise Webots would be started twice)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUPERVISOR_DIR="$SCRIPT_DIR/../C2_supervisor"
WORLD="${WORLD:-$SCRIPT_DIR/../../worlds/rmi_challenge_2.wbt}"      # <-- adjust to your world file

# The Webots Makefiles need WEBOTS_HOME
if [ -z "${WEBOTS_HOME:-}" ]; then
    for candidate in /usr/local/webots /usr/share/webots /opt/webots; do
        if [ -d "$candidate" ]; then
            export WEBOTS_HOME="$candidate"
            break
        fi
    done
fi
[ -n "${WEBOTS_HOME:-}" ] || echo "[setup] WARNING: WEBOTS_HOME is not set, make may fail" >&2

if [ ! -d "$SUPERVISOR_DIR" ]; then
    echo "[setup] ERROR: supervisor folder not found: $SUPERVISOR_DIR" >&2
    exit 1
fi

echo "[setup] Compiling the C2 supervisor in $SUPERVISOR_DIR"
( cd "$SUPERVISOR_DIR" && make clean && make )
echo "[setup] C2 supervisor compiled"

if [ "${1:-}" = "--no-run" ]; then
    exit 0
fi

if [ ! -f "$WORLD" ]; then
    echo "[setup] ERROR: world file not found: $WORLD (set WORLD=/path/to/world.wbt)" >&2
    exit 1
fi
if ! command -v webots >/dev/null 2>&1; then
    echo "[setup] ERROR: 'webots' command not found in PATH" >&2
    exit 1
fi

echo "[setup] Opening Webots on $WORLD"
exec webots --mode=realtime "$WORLD"
