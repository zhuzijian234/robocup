@echo off
rem ============================================================
rem  RoboCup telemetry host tool launcher  (double-click to run)
rem  ASCII-only on purpose: cmd.exe uses codepage 936 (GBK) and
rem  would garble any UTF-8 Chinese text written in this file.
rem ============================================================
setlocal
cd /d "%~dp0"

set "PY="
if exist "D:\anaconda3\anaconda3\python.exe" set "PY=D:\anaconda3\anaconda3\python.exe"
if not defined PY if exist "%USERPROFILE%\anaconda3\python.exe" set "PY=%USERPROFILE%\anaconda3\python.exe"
if not defined PY if exist "C:\ProgramData\anaconda3\python.exe" set "PY=C:\ProgramData\anaconda3\python.exe"
if not defined PY (
  rem Fall back to PATH order, but VERIFY each candidate actually runs:
  rem the WindowsApps "python.exe" stub is on PATH and is not a real Python.
  for /f "delims=" %%i in ('where python 2^>nul') do (
    if not defined PY (
      "%%i" -c "import sys" >nul 2>&1 && set "PY=%%i"
    )
  )
)

if not defined PY (
  echo [ERROR] No usable Python found.
  echo Install Anaconda or Python 3 ^(with "Add to PATH"^), then run this again.
  pause
  exit /b 1
)

echo Using Python: %PY%
"%PY%" -c "import serial" 2>nul
if errorlevel 1 (
  echo Installing pyserial ^(one-time, few seconds^) ...
  "%PY%" -m pip install pyserial -i https://pypi.tuna.tsinghua.edu.cn/simple
  if errorlevel 1 (
    echo [ERROR] pyserial install failed. Check network, or run manually:
    echo     "%PY%" -m pip install pyserial
    pause
    exit /b 1
  )
)

"%PY%" "%~dp0robocup_gui.py"
if errorlevel 1 (
  echo.
  echo [ERROR] The GUI exited with an error. Read the message above.
  pause
)
endlocal
