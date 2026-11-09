@echo off
cd /d "%~dp0"
py -m aerolap.gui
if errorlevel 1 pause
