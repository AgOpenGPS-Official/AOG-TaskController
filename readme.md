# AOG-TaskController 🚜

This is an experimental project to control sections of an ISOBUS implement using AgOpenGPS. It is based on the [AgIsoStack++](https://github.com/Open-Agriculture/AgIsoStack-plus-plus) library.

## Documentation

- [docs/LINUX_DAEMON.md](docs/LINUX_DAEMON.md) — running AOG-TaskController as a headless Linux service (Raspberry Pi, x86 SBCs, generic Linux). Install, systemd, CAN setup, troubleshooting.
- [docs/PROTOCOL.md](docs/PROTOCOL.md) — UDP wire protocol, PGN reference, settings/CLI, ISOBUS overview, and Python/C# example clients. Read this if you are writing an AgIO / AgValonia / third-party client.

## How to run the project

After installing the desired release of AOG-TaskController, you can run it directly through AgOpenGPS itself:

1. Open AgIO.
2. Go to the `Settings` tab.
3. Click on the `ISOBUS` tab.
4. Select the CAN adapter and channel you want to use.
5. Click on the `connect` button.

![how-to](resources/agopengps-howto.png)

## Raising tickets

Open %APPDATA% folder in explorer.
Zip all the folders (sometimes we need the ddop found in the subfolders) and the logs.
Attach them to the ticket.

## How to package the project

To package the project, you need to have the following tools installed:

- [CMake](https://cmake.org/download/)
- [C++ build tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)

Then, you can run the following commands:

```bash
mkdir build
cmake -S . -B build -DBUILD_EXAMPLES=OFF -DCMAKE_POLICY_VERSION_MINIMUM="3.16" -DBUILD_TESTING=OFF -Wno-dev
cmake --build build --config Release --target package
```

The installer will be generated in the `build` directory.

## Configuration

AOG-TaskController reads its configuration from a `settings.json` file located in:

- **Windows:** `%APPDATA%\AOG-TaskController\settings.json`
- **Linux:** `$XDG_CONFIG_HOME/AOG-TaskController/settings.json`, or `~/.config/AOG-TaskController/settings.json`
- **macOS:** `~/Library/Application Support/AOG-TaskController/settings.json`

### Available Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `subnet` | `int[3]` | `[192, 168, 5]` | LAN prefix used to select the AOG-facing NIC and broadcast address. |
| `tecuEnabled` | `bool` | `true` | Enables the internal Tractor ECU. Disable it when the tractor already has a TECU. A change made from the VT takes effect after restart. |
| `nmeaSendEnabled` | `bool` | `true` | Enables cyclic NMEA2000 COG/SOG transmission. Requires `tecuEnabled`. |
| `aogHeartbeatEnabled` | `bool` | `true` | Sends the 100 ms zero-section heartbeat when no implement is connected. Disable for AgOpenGPS versions before v6.8.2 beta 5. |
| `vtEnabled` | `bool` | `true` | Enables the ISOBUS Virtual Terminal UI. |
| `tcVersion` | `integer` | `3` | Task Controller version code, from `0` through `4`; `3` is Second Edition Draft. |
| `languageCode` | `string` | `"en"` | Two-character language code advertised on ISOBUS. |
| `countryCode` | `string` | `"US"` | Two-character country code advertised on ISOBUS. |

### Example `settings.json`

```json
{
  "subnet": [192, 168, 5],
  "tecuEnabled": true,
  "nmeaSendEnabled": true,
  "aogHeartbeatEnabled": true,
  "vtEnabled": true,
  "tcVersion": 3,
  "languageCode": "en",
  "countryCode": "US"
}
```

### Virtual Terminal compatibility

The VT object pool is embedded in the executable and automatically scales from its authored 480-pixel data mask and 80-pixel softkey designator to the connected terminal. It uses five virtual navigation softkeys. A VT with fewer than five physical keys must support softkey paging.

The application logs the detected VT version, screen size, softkey dimensions, and virtual/physical softkey counts. If a detected VT does not connect within 30 seconds, check those values and clear the terminal's stored/cached object pools before retrying.

## Task Controller Capabilities

- **ISO 11783-10 Version:** 2 (Second Edition)
- **Maximum Booms:** 1
- **Maximum Sections:** 64 (supports both individual sections and zone-based control)
- **Section Control:** Generation 1 (TC-SC) with support for DDI 160/161/290

## Contributing

Before committing it's better to run these commands: (requires the LLVM project to be installed)
 ```powershell
git ls-files | Select-String '\.(c|cc|cpp|cxx|h|hh|hpp|hxx|proto)$' | ForEach-Object {
    clang-format -i $_.ToString()
}
```

Install cmake-format with: 
```powershell
python -m pip install --upgrade pip
pip install cmake-format pyyaml
```
Then execute with:
```powershell
Get-ChildItem -Recurse -Filter CMakeLists.txt | ForEach-Object {
    python -m cmakelang.format -i $_.FullName
}
```
