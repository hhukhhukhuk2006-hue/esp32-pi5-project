#!/usr/bin/env bash
# runapp.sh — chạy GUI trên Linux / Raspberry Pi 5.
# Tương đương runapp.bat cho Windows.
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

if [ -x ".venv/bin/python" ]; then
    PY=".venv/bin/python"
else
    PY="python3"
fi

echo "==========================================="
echo " GUI Phuc hoi chuc nang - 1 joint KNEE"
echo " Using Python: $($PY --version)"
echo "==========================================="
exec "$PY" GUI.py "$@"
