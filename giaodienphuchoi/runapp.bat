@echo off
REM ────────────────────────────────────────────────────────────────────────────
REM  Giao diện phục hồi — PyQt5 adapted cho embedded computer (TWAI + CTC)
REM ────────────────────────────────────────────────────────────────────────────
REM  Set cwd về thư mục chứa GUI.py, rồi chạy.
REM  Không dùng venv riêng — dùng Python đã cài PyQt5 + numpy + pyserial.

cd /d "%~dp0scripts"
start "" pythonw.exe GUI.py