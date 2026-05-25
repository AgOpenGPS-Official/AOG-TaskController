# AOG-TaskController — Linux Daemon Deployment

This document covers running AOG-TaskController as a headless Linux service. It is the natural home for the TC when paired with AgValoniaGPS or any cross-platform AgOpenGPS variant, and it is the recommended deployment when the TC lives on a single-board computer (Raspberry Pi, NVIDIA Jetson, etc.) in the tractor cab.

For the UDP wire protocol used by AgIO / AgValonia / third-party clients, see [PROTOCOL.md](PROTOCOL.md).

---

## 1. Targets and CAN hardware

The Linux build produces a single statically-configured binary that runs in console mode (no GUI, no tray). It is built and released as a tarball by GitHub Actions for two architectures:

| Tarball | Target |
|---|---|
| `AOG-TaskController-linux-x86_64.tar.gz` | 64-bit Intel/AMD Linux (most desktops, NUCs, x86 fanless boxes) |
| `AOG-TaskController-linux-aarch64.tar.gz` | 64-bit ARM Linux (Raspberry Pi 3/4/5/Zero 2 W under 64-bit Raspberry Pi OS, Jetson, generic ARM SBCs) |

The TC reaches the ISOBUS via Linux SocketCAN (`--can_adapter=socketcan --can_channel=<iface>`). Linux has no built-in CAN controller on most SBCs, so you need one of:

- **SPI CAN HAT** based on MCP2515 (e.g. Waveshare RS485-CAN-HAT, PiCAN2). Driver is in mainline kernel, no userspace install. Comes up as `can0` after a small config tweak.
- **USB CAN adapter** (CANable v2, Innomaker USB2CAN, Geschwister Schneider gs_usb). Driver is in mainline. Comes up as `can0` when plugged in.

Either works. The TC does not care which — it only opens the `can0` (or whatever you pass via `--can_channel`) network interface.

---

## 2. Resource footprint

Measured on Ubuntu 24.04 aarch64 (Parallels VM, smoke-tested against `vcan0`):

| Resource | Steady state |
|---|---|
| CPU | <5% of one core. Main loop sleeps 1 ms between ticks; CAN I/O runs on its own thread. |
| RAM (RSS) | ~10–20 MB. |
| Disk | A few MB for the binary; optional log files under the config directory. |
| Network | <10 kbps UDP to AgIO/AgValonia. CAN bus usage depends on the implement. |

Pi Zero 2 W (4× Cortex-A53 @ 1 GHz, 512 MB RAM) handles this trivially. The bottleneck on a Pi deployment is the CAN adapter setup, not the TC itself.

---

## 3. Installation

### From a release tarball

```bash
ARCH=$(uname -m)   # "x86_64" or "aarch64"
TARBALL=AOG-TaskController-linux-${ARCH}.tar.gz

# Download the artifact attached to the GitHub Release you want
curl -LO https://github.com/AgOpenGPS-Official/AOG-TaskController/releases/latest/download/${TARBALL}
tar -xzf "${TARBALL}"
cd AOG-TaskController-linux-${ARCH}

# Install (as root)
sudo install -m 0755 bin/AOG-TaskController /usr/local/bin/AOG-TaskController
sudo install -m 0644 lib/systemd/system/aog-taskcontroller.service /etc/systemd/system/aog-taskcontroller.service
sudo install -m 0644 -D share/AOG-TaskController/settings.sample.json /etc/aog-taskcontroller/settings.sample.json
```

### Building from source

```bash
sudo apt-get install -y build-essential cmake ninja-build
git clone https://github.com/AgOpenGPS-Official/AOG-TaskController.git
cd AOG-TaskController
cmake -S . -B build -G Ninja -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -Wno-dev
cmake --build build
sudo install -m 0755 build/AOG-TaskController /usr/local/bin/AOG-TaskController
```

Boost and AgIsoStack are fetched automatically via CMake `FetchContent` — no system Boost required.

---

## 4. Configure the CAN interface

### MCP2515 HAT (SPI)

Add to `/boot/firmware/config.txt` (Pi 5/Bookworm) or `/boot/config.txt` (older):

```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
```

Reboot, then bring the interface up:

```bash
sudo ip link set can0 up type can bitrate 250000
ip -details link show can0
```

ISOBUS bitrate is 250 kbps. To make it survive reboots, add this to `/etc/systemd/network/80-can0.network` or call `ip link set` from a unit file that runs before `aog-taskcontroller.service`.

### USB CAN adapter

Most USB adapters using the `gs_usb` kernel driver appear as `can0` automatically. Bring it up the same way:

```bash
sudo ip link set can0 up type can bitrate 250000
```

### Virtual CAN (for development without hardware)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Run the TC with `--can_channel=vcan0`. Useful for protocol-level testing without an implement.

---

## 5. settings.json

The TC reads `settings.json` from `$XDG_CONFIG_HOME/AOG-TaskController/` (typically `~/.config/AOG-TaskController/` for an interactive user, or `/var/lib/aog/.config/AOG-TaskController/` when running under the `aog` system user).

```json
{
    "subnet": [192, 168, 5],
    "tecuEnabled": true,
    "aogHeartbeatEnabled": true
}
```

| Key | Type | Default | Meaning |
|---|---|---|---|
| `subnet` | `int[3]` | `[192, 168, 5]` | First three octets of the LAN the AgIO/AgValonia host lives on. TC enumerates NICs (via `getifaddrs(3)`) and binds the one whose IP starts with these octets. AgIO can override at runtime via the subnet-detection PGN. |
| `tecuEnabled` | `bool` | `true` | Whether the TC also impersonates a Tractor ECU on the bus (Speed Messages, NMEA2000 COG/SOG, BasicTractorECUServer announce). Set `false` if the tractor already has a TECU on the harness. |
| `aogHeartbeatEnabled` | `bool` | `true` | Whether the TC sends a 100 ms heartbeat UDP packet to AgIO/AgValonia even when no implement is connected. **Set to `false` if pairing with AgOpenGPS pre-v6.8.2 beta 5** — older AOG versions choke on the heartbeat. |

See [PROTOCOL.md](PROTOCOL.md) for the full reference.

---

## 6. The systemd unit

The release tarball ships `lib/systemd/system/aog-taskcontroller.service`:

```ini
[Unit]
Description=AOG-TaskController (ISOBUS TC for AgOpenGPS)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=aog
ExecStart=/usr/local/bin/AOG-TaskController --can_adapter=socketcan --can_channel=can0 --log_level=info --log2file
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Tweak `User=`, `--can_channel=`, and `--log_level=` for your deployment. Then:

```bash
sudo useradd --system --home-dir /var/lib/aog --create-home aog
sudo systemctl daemon-reload
sudo systemctl enable --now aog-taskcontroller.service
sudo systemctl status aog-taskcontroller.service
journalctl -u aog-taskcontroller -f
```

When the unit is active, `--log2file` writes to `/var/lib/aog/.config/AOG-TaskController/logs/`.

### Letting `aog` configure CAN

`ip link set` requires `CAP_NET_ADMIN`. The two clean options:

1. Bring `can0` up at boot from a separate root-owned unit, **before** `aog-taskcontroller.service` starts. Recommended.
2. Give the `aog` user permission via `AmbientCapabilities=CAP_NET_ADMIN` in the unit (less clean; the TC binary doesn't need to configure CAN itself — it just opens the interface).

---

## 7. Networking notes

- TC binds UDP **8888** on the LAN interface that matches `subnet`, **and** on `0.0.0.0:8888` for subnet detection.
- TC sends to UDP **9999** on the broadcast address of the matched subnet (e.g. `192.168.5.255:9999`).
- Firewalls: open inbound UDP 8888 (for AgIO/AgValonia → TC) and allow outbound UDP 9999 broadcast.
- Wi-Fi on Pi Zero 2 W is 2.4 GHz single-antenna. In a metal cab with electrical noise, you may want a Pi 4/5 or a wired link for production. Pi Zero 2 W is fine for development and bench testing.

---

## 8. Troubleshooting

**TC starts but doesn't see AgIO/AgValonia**
Check `subnet` in `settings.json` matches the LAN. The TC logs `Available IP addresses (getifaddrs):` at startup — verify your LAN NIC appears there and is in `subnet`.

**`bind: Address already in use`**
Something else is already on UDP 8888. The TC sets `SO_REUSEADDR` so two TCs on the same port won't fight, but a different daemon (or a stale TC) will conflict.

**No CAN traffic**
- `ip -details link show can0` should show `UP`. If it's `DOWN`, run `sudo ip link set can0 up type can bitrate 250000`.
- `candump -t a can0` should print frames if anything is on the bus.
- `dmesg | grep -i can` shows controller errors (bus-off, overruns) — most often a wiring or termination problem.

**TC claims address 247 instead of the preferred 233**
The address-claim arbitration walked up because either (a) the bus is empty and AgIsoStack's fallback range was used, or (b) something else has the preferred address. Look for an address-claim NACK in `--log_level=debug` output.

**TECU collisions**
If the tractor's factory TECU is on the bus, set `tecuEnabled: false` to avoid duplicate Speed Messages and NMEA2000 broadcasts. The TC will log `[Info] Tractor ECU disabled in settings...` at startup.

---

## 9. Where the logs live

| Scenario | Location |
|---|---|
| Interactive run | stdout of the terminal. Add `--log2file` to also write to `~/.config/AOG-TaskController/logs/AOG-TaskController_YYYY-M-D_H-M.log`. |
| systemd unit | `journalctl -u aog-taskcontroller`. With `--log2file`, also `/var/lib/aog/.config/AOG-TaskController/logs/`. |

Log levels (lowest to highest): `debug`, `info`, `warning`, `error`, `critical`. Default is whatever AgIsoStack defaults to; set explicitly with `--log_level=`.
