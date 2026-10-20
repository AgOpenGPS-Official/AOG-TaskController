# Sample implements

Real-world Device Descriptor Object Pools (DDOPs), captured from implements that were connected to the
AOG-TaskController and shared in issues. They are the input for tests that exercise DDOP parsing, section
geometry and section-control method detection without needing the machine.

`manifest.csv` is the machine-readable index. The table below is generated from it.

## Layout

One folder per implement type, using the same categories as the "Submit a machine" issue template:

| Folder | Contents |
|---|---|
| `sprayer/` | Sprayers |
| `spreader/` | Fertilizer spreaders |
| `seeder/` | Seed drills and single-product planters |
| `combined/` | Multi-product machines (e.g. planter + fertilizer + microgranulate) |
| `tractor/` | Tractor pools. **No sections**, so they must never register as an implement |
| `unsorted/` | Real pools whose machine could not be identified from the DDOP or the issue. Move them once known |

File names are `<BRAND>_<MODEL>[_<variant>].ddop`. The variant carries whatever distinguishes two pools of
the same machine: the section count (the Sulky reports a different pool depending on its configuration),
the section-control method, or the language. Put a `.iop` next to its `.ddop` under the same base name if a
machine's terminal pool is ever available.

## Columns

- **geometry**: where section offsets and width come from. `static` means the DDOP carries the values
  (DeviceProperty), so they are available at activation. `process-data` means the DDOP only declares the
  DDI, and the value has to be requested from the implement over the bus after activation. `none` is a pool
  without sections.
- **section_control**: the method the TC picks, in priority order: `DDI290` (condensed setpoint), `DDI161`
  (settable condensed actual), `DDI289` (per-element setpoint work state, actual on DDI 141 of the same element),
  `DDI141` (settable per-element work state). `none` means the TC cannot control it. Only process data that a
  device element links counts, because some implements also send objects no element references. The
  `section_control_method` test checks every file against this column.
- **connector_types**: DDI 157 values found on the connector elements (0 unknown, 1 drawbar,
  2 semi-mounted three-point, 3 mounted three-point, 4 hitch hook, ...).

## Privacy

The serial number string and the identity-number bits of the DVC NAME were zeroed in every file, with the
length and layout unchanged. Nothing else was altered. Do the same to anything you add: some pools carry a
VIN-style serial.

## Adding a machine

Attach the DDOP to a "Submit a machine" issue, then add it here with its issue number in the `source`
column. Only add pools from a machine you can name, and keep `unsorted/` for the ones you cannot.

## Synthetic pools

Files ending in `_synthetic.ddop` were not captured from a machine. They are derived from a real capture by the
implement firmware's own rules, to cover configurations nobody has captured yet. Replace one once a real capture of
that configuration exists.

The three MIDI 3 variants come from `ME_SPRITZE_SPR_MIDI_3_11sec.ddop` (firmware SPR_MIDI_3 V07.10.05.33), one per
section-control setting: old or new SC DDIs, condensed work state on or off. The firmware sends every template
object and only filters the element child lists, so these pools hold process data that no element links. The same
rules rebuild the real capture byte for byte.

## Files


### sprayer

| File | Sections | Geometry | Section control | Connector | Source | Note |
|---|---|---|---|---|---|---|
| `AGL_SPRAYER_7sec.ddop` | 7 | static | DDI290 | 1 | ddops.zip (no issue) | Y + width as static properties, no X offset. |
| `JD_740_SPRAYER_9sec.ddop` | 9 | process-data | DDI161 | 1 | #34 | Settable DDI 161. Geometry only as process data (Y + width). |
| `ME_ECO_SPRITZE_SC_BOX_9sec.ddop` | 9 | static | DDI141 | - | #64 | Mueller SC-Box from 2006. Per-section DDI 141 elements, no condensed DDIs. Static geometry. |
| `ME_SPRITZE_SPR_MIDI_3_11sec.ddop` | 11 | static | DDI290 | 0 | bench capture 2026-09-20 (no issue) | Full static geometry (DPT). Best reference pool. |
| `ME_SPRITZE_SPR_MIDI_3_11sec_newSC_perElement_synthetic.ddop` | 11 | static | DDI289 | 0 | synthetic, derived from the bench capture by firmware rules | SYNTHETIC, never seen on the wire. ISO options: new SC DDIs, condensed work state off. Each section links settable DDI 289 + DDI 141. |
| `ME_SPRITZE_SPR_MIDI_3_11sec_oldSC_CWS_synthetic.ddop` | 11 | static | DDI161 | 0 | synthetic, derived from the bench capture by firmware rules | SYNTHETIC, never seen on the wire. ISO options: old SC DDIs, condensed work state on. The boom links a settable DDI 161; the original DDI 161 objects stay in the pool, unlinked and not settable. |
| `ME_SPRITZE_SPR_MIDI_3_11sec_oldSC_perElement_synthetic.ddop` | 11 | static | DDI141 | 0 | synthetic, derived from the bench capture by firmware rules | SYNTHETIC, never seen on the wire. ISO options: old SC DDIs, condensed work state off. Each section links a settable DDI 141 (translated from 289). |

### spreader

| File | Sections | Geometry | Section control | Connector | Source | Note |
|---|---|---|---|---|---|---|
| `KVERNELAND_EDW2_ISO_2015_2sec.ddop` | 2 | process-data | DDI290 | - | #60 | Kept as kverneland_en.ddop in the DDOP editor folder. Issue #60 pairs it with the Exacta (a fertilizer spreader); model not confirmed. |
| `SULKY_ISOLINK_SPREADER_12sec.ddop` | 12 | process-data | DDI290 | - | #31, #93 | Same client NAME as the 6 section variant: the DDOP changes with machine configuration. Geometry only as process data. |
| `SULKY_ISOLINK_SPREADER_6sec.ddop` | 6 | process-data | DDI290 | - | #51 | Geometry only as process data. Localization label carries a control character (see #61). |

### seeder

| File | Sections | Geometry | Section control | Connector | Source | Note |
|---|---|---|---|---|---|---|
| `AGISOSTACK_EXAMPLE_SEEDER_6sec.ddop` | 6 | static | DDI290 | 9 | synthetic (AgIsoStack example) | Not a real machine. Full static geometry. |
| `KUHN_ESPRO_2sec_DDI141_en.ddop` | 2 | process-data | DDI141 | 2 | #10 | Same machine as the hu file, English strings. |
| `KUHN_ESPRO_2sec_DDI141_hu.ddop` | 2 | process-data | DDI141 | 2 | #10 | Non-ASCII strings (hu). Geometry only as process data. |
| `KUHN_ESPRO_2sec_DDI290_hu.ddop` | 2 | process-data | DDI290 | 2 | ddops.zip 2026-09-04 (#10 machine) | Newer capture of the same machine, condensed setpoint DDI 290 instead of 141. |
| `SKY_EASYDRILL_E2_1sec.ddop` | 1 | process-data | DDI290 | - | ddops.zip (no issue) | Single section, geometry only as process data. |

### combined

| File | Sections | Geometry | Section control | Connector | Source | Note |
|---|---|---|---|---|---|---|
| `AMAZONE_PRECEA_4500_6sec_2booms.ddop` | 6 | process-data | DDI290 | - | #35 | Planter + fertilizer + microgranulate. Only element 4 allows section control. |

### tractor

| File | Sections | Geometry | Section control | Connector | Source | Note |
|---|---|---|---|---|---|---|
| `JD_6000M_TRACTOR.ddop` | 0 | none | DDI141 | 1 3 4 5 6 7 8 | DDOP editor collection (woifi_tractor_en.ddop) | No sections. |
| `JD_6R150_TRACTOR.ddop` | 0 | none | none | 1 3 | #31, #93 | No sections. Must never register as an implement. |
| `JD_6R150_TRACTOR_minimal.ddop` | 0 | none | none | - | #93 | Reduced pool (no connectors, no booms). |
| `JD_6R175_6000RW_TRACTOR.ddop` | 0 | none | DDI141 | 1 3 4 5 6 7 8 | #34 | No sections, 8 connectors of mixed types. |

### unsorted

| File | Sections | Geometry | Section control | Connector | Source | Note |
|---|---|---|---|---|---|---|
| `MFG89_EDW2_ISO_DEMO_64sec_5booms.ddop` | 64 | process-data | DDI290 | - | ddops.zip 2026-04-06 | Stress case: 64 sections over 5 booms. |
| `MFG89_EDW3_MAIN_DEMO_24sec_3booms.ddop` | 24 | process-data | DDI290 | - | ddops.zip 2026-04-06 |  |
| `MFG89_ESA_FC_DEMO_2sec.ddop` | 2 | process-data | DDI141 | - | ddops.zip 2026-04-06 |  |
| `MFG89_E_COM_UD_DEMO_1sec_2booms.ddop` | 1 | process-data | DDI290 | - | ddops.zip 2026-04-06 |  |
| `MFG89_IXSPRAY_DEMO_7sec.ddop` | 7 | process-data | DDI290 | - | ddops.zip 2026-04-06 | Name suggests a sprayer. |
| `MFG89_PSD_MB_DEMO_12sec.ddop` | 12 | process-data | DDI290 | - | ddops.zip 2026-04-06 |  |
| `MFG89_PWRW_II_ISO_SIM_16sec.ddop` | 16 | process-data | DDI290 | - | ddops.zip 2026-04-06 | 16 sections in one pool. |
