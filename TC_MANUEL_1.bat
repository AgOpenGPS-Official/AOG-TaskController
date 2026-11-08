@echo off
REM Remplacer l'adresse IP par celle du Teensy ISOBridge.
REM Function Instance 1 = TC numero 2 (la numerotation commence a zero).
AOG-TaskController.exe --can_adapter=ISOBridge --can_channel=192.168.5.126 --tc_number_mode=manual --tc_function_instance=1 --log_level=debug --log2file
pause
