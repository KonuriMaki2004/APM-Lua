# APM-Lua

面向 ArduPilot / ArduCopter 的 Lua 脚本集合，用于在固件运行时扩展飞行行为。

> ## ⚠️ 适用范围声明
> **所有 Lua 脚本均仅通过相应版本固件实机测试，若为其他版本固件，可能不会生效。**

## 仓库用途

本仓库用于存放为 APM 固件（ArduPilot / ArduCopter）编写的 Lua 脚本。脚本通过 ArduPilot 内置的 Lua 脚本引擎在飞控上运行，在不修改固件源码的前提下扩展或约束飞行行为，例如按飞行高度动态限制手动下降速度。

仓库只收录可直接复制到飞控 SD 卡 `APM/scripts/` 目录使用的脚本，不包含固件源码、地面站配置或第三方依赖。

## 目标固件版本

- 适用固件：ArduCopter 4.6.3 与 4.7.0（多旋翼）。
- 参数适配：4.6.x 使用 `PILOT_SPEED_DN` / `PILOT_SPEED_UP`（单位 cm/s）；4.7.x 使用 `PILOT_SPD_DN` / `PILOT_SPD_UP`（单位 m/s）。脚本内置适配器自动识别，要求两组参数中有且仅有一组存在，否则只监视、不写入。
- 其他版本：脚本未做兼容处理。参数名称或语义一旦变化，脚本可能失效或行为不符预期。

## 目录结构

```text
.
├── LICENSE
├── README_CN.md / README_EN.md / README_FR.md / README_RU.md / README_ES.md / README_AR.md
└── scripts
    └── 近地面减速_near_ground_slowdown
        ├── near_ground_slowdown.lua
        ├── near_ground_slowdown_stage2_0p2.lua
        └── README.md
```

## 脚本清单

| 脚本 | 适用固件 | 功能概述 |
| --- | --- | --- |
| `near_ground_slowdown.lua` | ArduCopter 4.6.3 / 4.7.0 | 在 AltHold / Loiter 模式下按相对 home 点高度限制手动下降速度：2 m–5 m 限 0.50 m/s，≤ 2 m 限 0.10 m/s |
| `near_ground_slowdown_stage2_0p2.lua` | ArduCopter 4.6.3 / 4.7.0 | 同上，但 ≤ 2 m 档放宽至 0.20 m/s |

各脚本的参数定义、状态码与日志字段，详见对应功能目录下的 `README.md`。

## 使用须知

- 将脚本复制到飞控 SD 卡 `APM/scripts/` 目录，并确认 `SCR_ENABLE = 1`。
- 同一功能只加载一份脚本；`near_ground_slowdown` 的两个变体共用 `NGC_` 参数表，不可同时加载。
- 脚本只写入下降速度参数，不发送 arm / disarm / 电机 / 模式指令；落地依赖固件原生 `DISARM_DELAY` 自动上锁。
- 高度取自 EKF 相对 home 点的位置，不是对地真实高度；地形起伏与气压漂移会影响判定。
- 首次使用请拆桨在地面验证参数记录与还原行为，再进行实飞。

## 许可

本项目采用 GNU General Public License v3.0 授权，详见 `LICENSE`。

## 语言 / Languages

- 中文：[README_CN.md](README_CN.md)
- English：[README_EN.md](README_EN.md)
- Français：[README_FR.md](README_FR.md)
- Русский：[README_RU.md](README_RU.md)
- Español：[README_ES.md](README_ES.md)
- العربية：[README_AR.md](README_AR.md)
