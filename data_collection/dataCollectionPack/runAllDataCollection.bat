@echo off
REM Equivalent batch script for running data collection on Windows

REM Define duration variable (in seconds) for all data collection
set duration=20

REM Define global postfix for all filenames (e.g., experiment ID, date, condition)
set postfix=_
FOR /F "delims=" %%S IN ('py -c "import time,sys; sys.stdout.write('{0:.6f}'.format(time.time()))"') DO SET start_time=%%S
echo Using shared start time: %start_time%

REM start "FBGS" cmd /k "readFBGS.exe %duration% dataFBGS%postfix%.csv"
start "ATI FT" cmd /k "py readATIFT.py %duration% dataATIFT%postfix%.csv --start-time %start_time%"
REM start "Vicon" cmd /k "py readVicon.py %duration% dataVicon%postfix%.csv --start-time %start_time%"
start "Mark10 COM4" cmd /k "py readMark10.py %duration% dataMark10_1%postfix%.csv --port COM4 --start-time %start_time%"
start "Mark10 COM5" cmd /k "py readMark10.py %duration% dataMark10_2%postfix%.csv --port COM5 --start-time %start_time%"
@REM start "Motor1 Ramp" cmd /k "py readMotor1.py %duration% dataMotor1%postfix%.csv --motor-id 3 --increment-deg 30 --start-time %start_time%"
@REM start "Motor Sine" cmd /k "py readMotorFunction.py %duration% dataMotorFunction%postfix%.csv --motor-id 1 --amplitude 20 --frequency 0.5 --delay 2 --start-time %start_time%"
start "Motor Circle" cmd /k "py readMotorCircle.py %duration% dataMotorCircle%postfix%.csv --motor1-id 3 --motor2-id 4 --radius 20 --start-time %start_time%"
echo Data collection processes started...
echo Press any key to exit this window after all processes complete.
pause
