@echo off
echo --------------------------------------------------
echo Setting Virtual Environment
echo --------------------------------------------------

REM Create a virtual environment
python -m venv venv


echo --------------------------------------------------
echo Installing necessary packages
echo --------------------------------------------------

REM Activate the virtual environment
call venv\Scripts\activate.bat

REM Install pandas
pip install -r requirements.txt

echo --------------------------------------------------
echo Done!

REM Pause to keep the command prompt open after execution
pause
