@echo off
chcp 65001 >nul
title Dobot CR10A - AI Server (UDP 5005 / HTTP 5000)
cd /d "%~dp0"

set "CONDA_ENV=D:\Program Files\miniconda3\envs\env"
set "PATH=%CONDA_ENV%;%CONDA_ENV%\Library\bin;%CONDA_ENV%\Scripts;%PATH%"

echo ============================================================
echo  Dang khoi dong AI Server Dobot CR10A ...
echo  UDP : 127.0.0.1:5005
echo  HTTP: http://127.0.0.1:5000
echo  Tat server: nhan Ctrl + C
echo ============================================================

"%CONDA_ENV%\python.exe" -u "ai_src\step3_3d_trajectory\run_server.py" --host 127.0.0.1 --udp-port 5005 --http-port 5000

echo.
echo Server da dung.
pause
