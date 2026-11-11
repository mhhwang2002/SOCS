@echo off
setlocal
pushd "%~dp0..\.."
set "PYTHONPATH=src;.venv\Lib\site-packages"
set "AGV_PYTHON=C:\Program Files\Blender Foundation\Blender 5.2\5.2\python\bin\python.exe"
if not exist "%AGV_PYTHON%" set "AGV_PYTHON=python"
"%AGV_PYTHON%" -m examples.AGVSystem.run_panda %*
if errorlevel 1 pause
popd
