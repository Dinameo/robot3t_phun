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

call :PrepareZivid

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

:PrepareZivid
set "ZIVID_BIN=C:\Program Files\Zivid\bin"
if not exist "%ZIVID_BIN%\ZividCore.dll" (
    echo Chua co Zivid SDK. Van co the nhan dien tu file PLY.
    exit /b 0
)

set "ZIVID_SDK_VERSION="
for /f "usebackq delims=" %%V in (`powershell.exe -NoProfile -Command "$v = (Get-Item -LiteralPath (Join-Path $env:ZIVID_BIN 'ZividCore.dll')).VersionInfo.ProductVersion; if ($v -match '^\d+\.\d+\.\d+') { $Matches[0] }"`) do set "ZIVID_SDK_VERSION=%%V"
if not defined ZIVID_SDK_VERSION (
    echo Khong doc duoc phien ban Zivid SDK. Chuc nang PLY van su dung duoc.
    exit /b 0
)

echo Zivid SDK: %ZIVID_SDK_VERSION%
"%PYTHON_EXE%" -c "from importlib.metadata import version; import os, sys; sys.exit(version('zivid').split('.')[:3] != os.environ['ZIVID_SDK_VERSION'].split('.'))" >nul 2>&1
if errorlevel 1 (
    echo Dang cai goi Python Zivid khop voi SDK %ZIVID_SDK_VERSION%...
    "%PYTHON_EXE%" -m pip install "zivid==%ZIVID_SDK_VERSION%.*"
    if errorlevel 1 (
        echo Cai Zivid that bai. Kiem tra mang va pip. Van co the nhan dien tu file PLY.
        exit /b 0
    )
)

set "PATH=%ZIVID_BIN%;%PATH%"
"%PYTHON_EXE%" -c "import os; dll_dir = os.add_dll_directory(os.environ['ZIVID_BIN']); import zivid; print('Zivid Python: OK')"
if errorlevel 1 echo Khong nap duoc Zivid Python. Kiem tra SDK va DLL. Van co the nhan dien tu file PLY.
exit /b 0
