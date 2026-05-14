# AOG-TaskController 🚜

This is an experimental project to control sections of an ISOBUS implement using AgOpenGPS. It is based on the [AgIsoStack++](https://github.com/Open-Agriculture/AgIsoStack-plus-plus) library.

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

- **Windows:** `%APPDATA%\AOG-Taskcontroller\settings.json`

### Available Settings

#### `tecuEnabled`
- **Type:** Boolean
- **Default:** `true`
- **Description:** Enables the internal Tractor ECU (TECU) simulator. When enabled, AOG-TaskController broadcasts TECU speed messages on the ISOBUS.
- **⚠️ Important:** 
  - Disable if your tractor already has a TECU to avoid conflicts (two TECUs on the same bus will cause issues)
  - Enable when your tractor lacks a TECU
- **TECU Speed Messages Provided (when enabled):**
  - **Ground-based Speed** (PGN 65256, 0xFEE8) - 100ms interval
  - **Wheel-based Speed** (PGN 65256, 0xFEE8) - 100ms interval  
  - **Machine Selected Speed** (PGN 65256, 0xFEE8) - 100ms interval
  - **Control Function Functionalities** (PGN 64654, 0xFC8E) - Announces TECU Class 1 capability (no control functions)
  - **NMEA2000 COG/SOG** - Optional navigation data

#### `aogHeartbeatEnabled`
- **Type:** Boolean  
- **Default:** `true`
- **Description:** Enables the heartbeat message sent to AgOpenGPS every 100ms. This allows AOG to detect when the ISOBUS TC is running and display the ISOBUS button status.
- **⚠️ Important:** If using AgOpenGPS **pre-v6.8.2 beta 5**, set this to `false` to avoid compatibility issues. Newer versions of AOG can properly handle the heartbeat message.

### Example `settings.json`

```json
{
  "subnet": [192, 168, 5],
  "tecuEnabled": true,
  "aogHeartbeatEnabled": true
}
```

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
