@echo off
REM Equivalent batch script for running data collection on Windows

REM Define duration variable (in seconds) for all data collection
set duration=30

REM Define output directory for all data files
set output_dir=20260310/static_test
REM circle_150_v_1_3rd_morepret
REM Create output directory if it doesn't exist
if not exist "%output_dir%" mkdir "%output_dir%"
echo Data will be saved to: %output_dir%

REM Define global postfix for all filenames (e.g., experiment ID, date, condition)
set postfix=
FOR /F "delims=" %%S IN ('py -c "import time,sys; sys.stdout.write('{0:.6f}'.format(time.time()))"') DO SET start_time=%%S
echo Using shared start time: %start_time%

REM ID MOTOR 1 -> - deg (couterclockwise) / MARK 10 COM 6
REM ID MOTOR 2 -> + deg (clockwise) / MARK 10 COM 7
REM ID MOTOR 3 -> - deg (couterclockwise) / MARK 10 COM4
REM ID MOTOR 4 -> + deg (clockwise) / MARK 10 COM5

REM ********************************    FBGS      ********************************
start "FBGS" cmd /k "readFBGS.exe %duration% %output_dir%\dataFBGS%postfix%.csv"

REM ********************************    ATI FT      ********************************
start "ATI FT" cmd /k "py readATIFT.py %duration% %output_dir%\dataATIFT%postfix%.csv --start-time %start_time%"

REM ********************************    Mocap      ********************************
@REM start "Vicon" cmd /k "py readVicon.py %duration% %output_dir%\dataVicon%postfix%.csv --start-time %start_time%"
start "OptiTrack" cmd /k "py optitrackPython.py %duration% %output_dir%\dataOptiTrack%postfix%.csv --start-time %start_time%"


REM ********************************    MARK 10      ********************************
start "Mark10 COM5" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_-y%postfix%.csv --port COM5 --start-time %start_time%"
start "Mark10 COM4" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_-x%postfix%.csv --port COM4 --start-time %start_time%"
start "Mark10 COM6" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_+x%postfix%.csv --port COM6 --start-time %start_time%"
start "Mark10 COM7" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_+y%postfix%.csv --port COM7 --start-time %start_time%"

REM ********************************    GYROSCOPE      ********************************
start "Gyro" cmd /k "..\sensors\gyros\build_spencer\Debug\sparkfun_ism330dhcx.exe %output_dir% 300 %duration%"

REM ********************************    MOTORS      ********************************
@REM start "Motor" cmd /k "py read4MotorCircle.py %duration% %output_dir%\dataMotor%postfix%.csv --motor1-id 1 --motor2-id 2 --motor3-id 3 --motor4-id 4 --start-time %start_time%"

echo Data collection processes started...
echo Press any key to exit this window after all processes complete.
pause
