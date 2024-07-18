@echo off

echo --------------------------------------------------
echo Running Main Script
echo --------------------------------------------------

set scriptPath=%~dp0
"%scriptPath%./venv/Scripts/python.exe" "%scriptPath%./main.py"

REM Pause to keep the command prompt open after execution
pause