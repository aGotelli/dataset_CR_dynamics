@echo off
REM Equivalent batch script for running data collection on Windows

REM Define duration variable (in seconds) for all data collection
set duration=5

REM Define output directory for all data files
set output_dir=reachPT_m1_-80_m2_120_v2
REM circle_150_v_1_3rd_morepret
REM Create output directory if it doesn't exist
if not exist "%output_dir%" mkdir "%output_dir%"
echo Data will be saved to: %output_dir%

REM Define global postfix for all filenames (e.g., experiment ID, date, condition)
set postfix=_
FOR /F "delims=" %%S IN ('py -c "import time,sys; sys.stdout.write('{0:.6f}'.format(time.time()))"') DO SET start_time=%%S
echo Using shared start time: %start_time%

REM ID MOTOR 1 -> - deg (couterclockwise) / MARK 10 COM 6
REM ID MOTOR 2 -> + deg (clockwise) / MARK 10 COM 7
REM ID MOTOR 3 -> - deg (couterclockwise) / MARK 10 COM4
REM ID MOTOR 4 -> + deg (clockwise) / MARK 10 COM5



@REM start "FBGS" cmd /k "readFBGS.exe %duration% dataFBGS%postfix%.csv"
start "ATI FT" cmd /k "py readATIFT.py %duration% %output_dir%\dataATIFT%postfix%.csv --start-time %start_time%"
start "Vicon" cmd /k "py readVicon.py %duration% %output_dir%\dataVicon%postfix%.csv --start-time %start_time%"

REM ********************************    MARK 10      ********************************
start "Mark10 COM5" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_2_-y%postfix%.csv --port COM5 --start-time %start_time%"
start "Mark10 COM4" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_1_-x%postfix%.csv --port COM4 --start-time %start_time%"
start "Mark10 COM6" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_1_x%postfix%.csv --port COM6 --start-time %start_time%"
start "Mark10 COM7" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_2_y%postfix%.csv --port COM7 --start-time %start_time%"

REM ********************************    GYROSCOPE      ********************************
@REM start "Gyro" cmd /k "..\sensors\gyros\build\Debug\sparkfun_ism330dhcx.exe gyros 300 %duration%"

REM ********************************    MOTORS      ********************************
@REM start "Motor1 Ramp" cmd /k "py readMotor1.py %duration% dataMotor1%postfix%.csv --motor-id 4 --increment-deg 40 --start-time %start_time%"
@REM start "Motor Sine" cmd /k "py readMotorFunction.py %duration% dataMotorFunction%postfix%.csv --motor-id 1 --amplitude 20 --frequency 0.5 --delay 2 --start-time %start_time%"
@REM start "Motor Circle" cmd /k "py readMotorCircle.py %duration% dataMotorCircle%postfix%.csv --motor1-id 3 --motor2-id 4 --radius 25 --start-time %start_time%"
@REM start "Motor Oscillation" cmd /k "py readMotorOscill.py %duration% %output_dir%\dataMotor_pair1%postfix%.csv --motor1-id 2 --motor2-id 4 --radius 40 --start-time %start_time%"
start "PontReach" cmd /k "py read4motorToPoint.py %duration% %output_dir%\dataMotor%postfix%.csv --motor1-id 1 --motor2-id 2 --motor3-id 3 --motor4-id 4 --increment-deg1 -80 --increment-deg2 120 --start-time %start_time%"
@REM start "Motor sequence" cmd /k "py readMotorCircle.py %duration% datasequence%postfix%.csv --motor1-id 1 --motor2-id 4 --start-time %start_time%"
@REM start "Four Motor Circle" cmd /k "py read4MotorCircle.py %duration% %output_dir%\datasequence%postfix%.csv --motor1-id 1 --motor2-id 2 --motor3-id 3 --motor4-id 4 --radius 150 --start-time %start_time%"

echo Data collection processes started...
echo Press any key to exit this window after all processes complete.
pause
