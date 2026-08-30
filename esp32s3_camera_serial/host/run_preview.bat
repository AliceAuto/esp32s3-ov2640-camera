@echo off
cd /d D:\Projects\ov2640\esp32s3_camera_serial\host
python -u preview_serial.py COM7 2000000 t 2>preview_err.log
pause
