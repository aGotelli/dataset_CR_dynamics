@echo off
REM Equivalent batch script for running data collection on Windows

REM Define duration variable (in seconds) for all data collection
set duration=30

REM Define output directory for all data files
set output_dir=20260310/static_test

REM Create output directory if it doesn't exist
if not exist "%output_dir%" mkdir "%output_dir%"
echo Data will be saved to: %output_dir%

REM ID MOTOR 1 -> - deg (couterclockwise) / MARK 10 COM 6
REM ID MOTOR 2 -> + deg (clockwise) / MARK 10 COM 7
REM ID MOTOR 3 -> - deg (couterclockwise) / MARK 10 COM4
REM ID MOTOR 4 -> + deg (clockwise) / MARK 10 COM5

REM ********************************    FBGS      ********************************
start "FBGS" cmd /k "readFBGS.exe %duration% %output_dir%\dataFBGS.csv"

REM ********************************    ATI FT      ********************************
start "ATI FT" cmd /k "py readATIFT.py %duration% %output_dir%\dataATIFT.csv"

REM ********************************    Mocap      ********************************
start "OptiTrack" cmd /k "py optitrackPython.py %duration% %output_dir%\dataOptiTrack.csv"


REM ********************************    MARK 10      ********************************
start "Mark10 COM5" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_-y.csv --port COM5"
start "Mark10 COM4" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_-x.csv --port COM4"
start "Mark10 COM6" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_+x.csv --port COM6"
start "Mark10 COM7" cmd /k "py readMark10.py %duration% %output_dir%\dataMark10_+y.csv --port COM7"

REM ********************************    MOTORS      ********************************
start "Motor" cmd /k "py read4MotorCircle.py %duration% %output_dir%\dataMotor.csv --motor1-id 1 --motor2-id 2 --motor3-id 3 --motor4-id 4"


REM ********************************    FT SENSOR      ********************************
@REM start "HEX12" cmd /k "py read_resense_ft.py %duration% %output_dir%\dataResenseFT%postfix%.csv"

echo Data collection processes started...
echo Press any key to exit this window after all processes complete.
pause
