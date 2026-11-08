# ISOBridge testing

The `ISOBridge` CAN driver reaches the ISOBUS through a Teensy ISOBridge over UDP (AC v1 protocol), so no USB CAN adapter is needed on the PC.

## Runtime arguments

- `--can_adapter=ISOBridge` selects the driver.
- `--can_channel=192.168.5.126` is the Teensy's IPv4 address, and is required. The UDP port is fixed at `9200`.

AgIO has no entry for this adapter yet, so start the TC yourself. For example, save this as `TC_ISOBRIDGE.bat` next to `AOG-TaskController.exe` and replace the IP address if needed:

```bat
@echo off
AOG-TaskController.exe --can_adapter=ISOBridge --can_channel=192.168.5.126 --log_level=debug --log2file
pause
```

The Teensy must have ISOBridge enabled and be reachable from the PC. Allow UDP traffic to and from port 9200 in Windows Firewall.

## Expected startup sequence

1. The TC opens a UDP socket on a free local port and sends HELLO packets to `<Teensy IP>:9200` for up to 4 seconds.
2. The Teensy replies with HELLO_ACK. If it doesn't, startup fails with `No HELLO_ACK from ISOBridge at ...`.
3. CAN frames are exchanged as DATA packets. The TC sends a keepalive HELLO every second, which also registers it again if the Teensy reboots.
4. The CAN bitrate stays at 250 kbit/s, set in the Teensy firmware.

## Test precautions

- Stop AgIO's Task Controller process before starting the batch file. Don't run two Task Controllers at the same time.
- First check the UDP handshake and that the bus is visible, without the implement if possible.
- The bridge accepts 4 clients. When its table is full, startup fails with `ISOBridge refused client registration`.
