# ISOBridge and TC instance testing

This build adds a direct UDP CAN driver for the Teensy ISOBridge AC v1 protocol.

## Runtime arguments

- `--can_adapter=ISOBridge` selects the Teensy bridge.
- `--can_channel=192.168.5.126` is the Teensy's IPv4 address. The UDP port is fixed at `9200` and is not an AgIO setting.
- `--tc_number_mode=auto` chooses the first unused Task Controller Function Instance from 0 through 7 after a 1.5 s bus discovery period.
- `--tc_number_mode=manual --tc_function_instance=1` forces Function Instance 1 (the second TC instance).
- If the TC options are omitted, the backward-compatible default is manual Function Instance 0.

## Windows test launchers

Create these files next to `AOG-TaskController.exe` (replace the IP if AgIO uses another one).

### TC_AUTO.bat

```bat
@echo off
AOG-TaskController.exe --can_adapter=ISOBridge --can_channel=192.168.5.126 --tc_number_mode=auto --log_level=debug --log2file
pause
```

### TC_MANUAL_1.bat

```bat
@echo off
AOG-TaskController.exe --can_adapter=ISOBridge --can_channel=192.168.5.126 --tc_number_mode=manual --tc_function_instance=1 --log_level=debug --log2file
pause
```

The Teensy must have ISOBridge enabled and be reachable from the PC. Allow UDP traffic to/from port 9200 in Windows Firewall.

## Expected startup sequence

1. The PC opens a UDP socket with an ephemeral local port and sends AC v1 HELLO packets to `<Teensy IP>:9200`.
2. The Teensy replies with HELLO_ACK. The plugin registers as an ISOBridge client and sends keepalive HELLO packets once per second.
3. CAN frames are exchanged as AC v1 DATA datagrams. The bridge's CAN bitrate remains 250 kbit/s, configured in the Teensy firmware.
4. After the CAN interface starts, TaskController listens for 1.5 seconds and logs Task Controllers already present on the bus.
5. Automatic mode chooses the first unused Function Instance. Manual mode uses the requested value.

## Important test precautions

- Stop AgIO's TaskController process before starting either batch file; do not run two TaskController instances simultaneously.
- First verify the UDP handshake and bus visibility without connecting the implement if possible.
- Keep the tractor's original VT/TC active during the discovery test, and confirm its VT still operates normally.
- `Function Instance` is part of the ECU NAME. It is not the CAN source address. Address claiming remains handled by AgIsoStack++ and J1939 arbitration.
- The automatic selector considers every online Control Function whose NAME function code is Task Controller, regardless of manufacturer, to avoid selecting an already-used instance.
- The TECU NAME is explicitly kept at Function Instance 0; only the Task Controller NAME uses the selected instance.
