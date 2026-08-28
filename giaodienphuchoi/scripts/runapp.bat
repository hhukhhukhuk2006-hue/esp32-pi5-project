@echo off
REM runapp.bat — chạy GUI chính (PyQt5 + ODrive)
REM Tương đương GUI/LLRR_app/scripts/runapp.bat cũ.
REM Gọi được cả từ cmd lẫn PowerShell (.\runapp.bat).

setlocal
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"

REM Activate venv nếu tồn tại; nếu không, dùng system python.
if exist ".venv\Scripts\python.exe" (
    set "PY=.venv\Scripts\python.exe"
) else (
    set "PY=python"
)

echo ===========================================
echo  GUI Phuc hoi chuc nang - 1 joint KNEE
echo  Using Python: %PY%
echo ===========================================
%PY% --version
%PY% GUI.py %*
endlocal

REM Nếu chạy bằng double-click từ Explorer, giữ cửa sổ cmd mở
if "%1"=="" pause
