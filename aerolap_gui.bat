@echo off
cd /d "%~dp0"
rem always run this folder's code (not an older installed copy)
set "PYTHONPATH=%~dp0src;%PYTHONPATH%"
py -m aerolap.gui
if errorlevel 1 pause
