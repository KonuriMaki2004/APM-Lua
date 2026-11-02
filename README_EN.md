# APM-Lua

A collection of Lua scripts for ArduPilot / ArduCopter that extend flight behaviour at firmware runtime.

> ## ⚠️ Applicability Notice
> **All Lua scripts have been tested on real hardware only with the corresponding firmware version; they may not take effect on any other firmware version.**

## Purpose of This Repository

This repository hosts Lua scripts written for APM firmware (ArduPilot / ArduCopter). The scripts run on the flight controller through the built-in ArduPilot Lua scripting engine and extend or constrain flight behaviour without modifying the firmware source code — for example, dynamically limiting manual descent speed as a function of flight altitude.

The repository contains only scripts that can be copied directly into the `APM/scripts/` directory of the flight controller SD card. It does not include firmware source code, ground control station configuration, or third-party dependencies.

## Target Firmware Versions

- Supported firmware: ArduCopter 4.6.3 and 4.7.0 (multirotor).
- Parameter adaptation: 4.6.x uses `PILOT_SPEED_DN` / `PILOT_SPEED_UP` (unit: cm/s); 4.7.x uses `PILOT_SPD_DN` / `PILOT_SPD_UP` (unit: m/s). A built-in adapter identifies the pair automatically and requires exactly one of the two pairs to be present; otherwise the script only monitors and never writes.
- Other versions: no compatibility handling is implemented. Any change in parameter names or semantics may render the scripts ineffective or cause unexpected behaviour.

## Directory Layout

```text
.
├── LICENSE
├── README_CN.md / README_EN.md / README_FR.md / README_RU.md / README_ES.md / README_AR.md
└── scripts
    └── 近地面减速_near_ground_slowdown
        ├── near_ground_slowdown.lua
        ├── near_ground_slowdown_stage2_0p2.lua
        ├── near_ground_slowdown_smooth_10m_stage2_0p2.lua
        ├── near_ground_slowdown_smooth_10m_stage2_0p2.md
        └── README.md
```

## Script Inventory

| Script | Firmware | Summary |
| --- | --- | --- |
| `near_ground_slowdown.lua` | ArduCopter 4.6.3 / 4.7.0 | In AltHold / Loiter, limits manual descent speed by height above the home point: 0.50 m/s between 2 m and 5 m, 0.10 m/s at or below 2 m |
| `near_ground_slowdown_stage2_0p2.lua` | ArduCopter 4.6.3 / 4.7.0 | Same as above, but the ≤ 2 m band is relaxed to 0.20 m/s |
| `near_ground_slowdown_smooth_10m_stage2_0p2.lua` | ArduCopter 4.6.3 / 4.7.0 | Same as above, plus a smooth 10–5 m buffer (curve converging from the original value to 0.50 m/s) and integer cm/s quantization for 4.6.x |

Parameter definitions, state codes and log fields for each script are documented in the `README.md` of the corresponding function directory.

## Usage Notes

- Copy the script into the `APM/scripts/` directory on the flight controller SD card and verify that `SCR_ENABLE = 1`.
- Load only one script per function; the three `near_ground_slowdown` variants share the `NGC_` parameter table and must not be loaded simultaneously.
- The scripts write descent-speed parameters only and never send arm / disarm / motor / mode commands; landing relies on the native `DISARM_DELAY` automatic disarm.
- Height is derived from the EKF position relative to the home point, not from true height above ground; terrain relief and barometric drift affect the decision.
- For first use, remove the propellers and verify parameter recording and restoration on the ground before flying.

## License

This project is licensed under the GNU General Public License v3.0. See `LICENSE`.

## Languages

- 中文：[README_CN.md](README_CN.md)
- English：[README_EN.md](README_EN.md)
- Français：[README_FR.md](README_FR.md)
- Русский：[README_RU.md](README_RU.md)
- Español：[README_ES.md](README_ES.md)
- العربية：[README_AR.md](README_AR.md)
