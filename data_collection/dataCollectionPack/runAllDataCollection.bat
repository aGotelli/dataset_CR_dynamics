@echo off
REM Equivalent batch script for running data collection on Windows

REM Define duration variable (in seconds) for all data collection
set duration=2

REM Define global postfix for all filenames (e.g., experiment ID, date, condition)
set postfix=_

REM start "FBGS" cmd /k "readFBGS.exe %duration% dataFBGS%postfix%.csv"
REM start "ATI FT" cmd /k "py readATIFT.py %duration% dataATIFT%postfix%.csv"
REM start "Vicon" cmd /k "py readVicon.py %duration% dataVicon%postfix%.csv"
REM start "Mark10 COM4" cmd /k "py readMark10.py %duration% dataMark10_1%postfix%.csv --port COM4"
start "Mark10 COM5" cmd /k "py readMark10.py %duration% dataMark10_2%postfix%.csv --port COM5"

echo Data collection processes started...
echo Press any key to exit this window after all processes complete.
pause