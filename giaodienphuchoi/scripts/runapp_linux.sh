#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
#  Giao diện phục hồi — chạy trên Raspberry Pi 5 / Linux
# ─────────────────────────────────────────────────────────────────────────────
#  Cách dùng:
#    chmod +x runapp_linux.sh
#    ./runapp_linux.sh                    ← chạy foreground
#    nohup ./runapp_linux.sh &            ← chạy background, sống qua SSH logout
#  Để auto-start khi Pi5 boot: xem README phần "systemd service"
# ─────────────────────────────────────────────────────────────────────────────

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
export PYTHONPATH="$PROJECT_ROOT:${PYTHONPATH}"

# Force Qt dùng platform plugin phù hợp cho framebuffer (khi không có X11)
# - eglfs: dùng cho màn hình cảm ứng DSI/HDMI không qua desktop
# - linuxfb: fallback cho framebuffer thuần
# - xcb: dùng khi chạy trên LXDE/XFCE desktop bình thường
if [ -z "$QT_QPA_PLATFORM" ]; then
    if [ -n "$DISPLAY" ]; then
        export QT_QPA_PLATFORM="xcb"
    elif [ -e /dev/dri/card0 ]; then
        export QT_QPA_PLATFORM="eglfs"
    else
        export QT_QPA_PLATFORM="linuxfb"
    fi
fi

# In diagnosis trước khi chạy
python3 -c "
import sys
sys.path.insert(0, '$SCRIPT_DIR')
import config
config.print_diagnosis()
"

echo "[runapp_linux] Starting GUI on $(date)"
cd "$SCRIPT_DIR"
exec python3 GUI.py "$@"