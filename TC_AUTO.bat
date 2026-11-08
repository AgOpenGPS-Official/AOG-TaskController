@echo off
REM Remplacer l'adresse IP par celle du Teensy ISOBridge.
AOG-TaskController.exe --can_adapter=ISOBridge --can_channel=192.168.5.126 --tc_number_mode=auto --log_level=debug --log2file
pause
