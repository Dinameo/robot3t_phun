@echo off
setlocal
chcp 65001 >nul
title Dobot CR10A - AI Server (UDP 5005 / HTTP 5000)
cd /d "%~dp0" || exit /b 1

set "PYTHON_EXE=C:\Users\nhan\miniconda3\envs\py312\python.exe"
if not exist "%PYTHON_EXE%" (
    echo Khong tim thay Python tai "%PYTHON_EXE%".
    echo Tao moi truong .venv va cai cac goi trong requirements.txt.
    pause
    exit /b 1
)

echo ============================================================
echo  Dang khoi dong AI Server Dobot CR10A ...
echo  UDP : 127.0.0.1:5005
echo  HTTP: http://127.0.0.1:5000
echo  Tat server: nhan Ctrl + C
echo ============================================================

"%PYTHON_EXE%" -u "ai_src\step3_3d_trajectory\run_server.py" --host 127.0.0.1 --udp-port 5005 --http-port 5000
set "SERVER_EXIT_CODE=%ERRORLEVEL%"

echo.
echo Server da dung.
pause
exit /b %SERVER_EXIT_CODE%