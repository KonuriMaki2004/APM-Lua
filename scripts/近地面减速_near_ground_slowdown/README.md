# 近地面减速 / Near-Ground Descent Speed Cap

面向 **ArduCopter 4.6.3 / 4.7.0** 的 Lua 脚本：在 **AltHold(2) / Loiter(5)** 模式下，按
**EKF 相对 home 点的高度**自动收紧飞行员手动下降速度参数，避免手动降落时下降过快砸地。

脚本只修改下降速度参数，**不发送 arm / disarm / 电机 / 模式指令**；落地靠固件原生
`DISARM_DELAY` 自动上锁。

> 高度取自 `ahrs:get_relative_position_NED_home()`，是**相对 home 点的高度，不是对地真实高度**。
> 地形起伏、气压漂移、home 点偏差都会影响判定结果。

---

## 目录内容 / Contents

| 文件 | 说明 |
| --- | --- |
| `near_ground_slowdown.lua` | 标准版：2 m 以下限 0.10 m/s |
| `near_ground_slowdown_stage2_0p2.lua` | 二阶段版：2 m 以下放宽到 0.20 m/s |
| `README.md` | 本说明文档 |

两份脚本功能同源，**只能加载其中一份**：它们使用同一张恢复参数表（table key `192`、
魔数 `602902`、前缀 `NGC_`），同时加载会互相覆盖恢复记录并争夺控制权。

---

## 限速档位 / Speed Bands

| 相对 home 高度 | `near_ground_slowdown.lua` | `near_ground_slowdown_stage2_0p2.lua` |
| --- | --- | --- |
| > 5 m | 不限制（使用原始参数） | 不限制（使用原始参数） |
| 2 m < 高度 ≤ 5 m | 0.50 m/s | 0.50 m/s |
| 高度 ≤ 2 m | 0.10 m/s | 0.20 m/s |
| 高度/数据故障 | 0.10 m/s | 0.10 m/s（`fault cap`） |

行为细节：

- **收紧立即生效**，**放松需持续 500 ms 且越过 0.3 m 滞回**，避免在档位边界反复抖动。
- 回调周期 100 ms（10 Hz）；两次回调间隔超过 500 ms 视为调度卡顿，本周期强制走保守档。
- 高度数据不可信（EKF 不健康、home 未设置、高度无效）或参数异常时，强制最紧档并告警。

---

## 适用固件与参数适配 / Firmware Adapter

| 固件 | 下降参数 | 上升参数 | 单位换算 |
| --- | --- | --- | --- |
| 4.6.x | `PILOT_SPEED_DN` | `PILOT_SPEED_UP` | cm/s，除以 100 |
| 4.7.x | `PILOT_SPD_DN` | `PILOT_SPD_UP` | m/s，除以 1 |

脚本要求**两组参数有且仅有一组存在**。两者都不存在或同时存在时，脚本报
`unsupported descent parameter adapter` 并只做监视，不写任何参数。

固件语义：`DN = 0` 表示“沿用 UP 的值”，脚本会自动按 UP 换算等效下降速度，并把 UP 一并纳入基线校验。

---

## 安装 / Installation

1. 将**其中一份** `.lua` 文件放入飞控 SD 卡的 `APM/scripts/` 目录（文件名可保持不变）。
2. 确认 `SCR_ENABLE = 1`，且 `SCR_HEAP_SIZE` 足够（脚本会注册 15 个参数、写飞行日志；
   若日志出现 `recovery table unavailable` 说明参数表/堆空间不足）。
3. 上电后检查地面站消息以 `NGS:` 开头的输出，确认脚本已运行。
4. 建议先在地面解锁前确认 `NGC_*` 参数已生成、`NGC_READY = 1`。

---

## 使用流程 / Workflow

**正常流程（地面加载）**

1. 地面未解锁加载脚本 → 脚本把当前下降参数记为基线，写入 `NGC_*` 恢复表。
2. 起飞后在 AltHold / Loiter 中自动按高度限速，其它模式（Auto / Land / RTL 等）交还固件原生逻辑。
3. 落地、`DISARM_DELAY` 自动上锁 → 脚本自动把参数还原为原始值，并重建基线。

**原始值未知（空中首次加载 / 记录损坏 / 读参失败）**

- 进入 `tighten only` 模式：**只允许收紧，绝不向上放大**，状态码 8。
- 落地后需人工核对/恢复原生下降参数，然后在**未解锁**状态把 `NGC_ACK` 置 `1`。
- 脚本只接受“先看到 0、再看到 1”的新上升沿：解锁期间写入的 ACK 会被清零并提示
  `NGC_ACK ignored while armed`；VM 启动时就存在的 ACK 也不被承认。

**`DISARM_DELAY` 为 0 或读不到时**：落地不会自动上锁，脚本每 10 s 告警一次
`DISARM_DELAY disabled/unknown; native auto-disarm unavailable`。

---

## 恢复参数表 `NGC_*` / Recovery Table

脚本以 `param:add_table(192, 'NGC_', 15)` 注册，字段顺序即参数索引 1..15：

| 索引 | 名称 | 含义 |
| --- | --- | --- |
| 1 | `NGC_VER` | 结构版本（当前 2） |
| 2 | `NGC_ADAPTER` | 参数适配器编号（46 / 47） |
| 3 | `NGC_BASE` | 原始下降参数值 |
| 4 | `NGC_UP` | 原始上升参数值 |
| 5 | `NGC_EXPECT` | 脚本认为参数当前应等于的值 |
| 6 | `NGC_PVAL` | 写前意图值 |
| 7 | `NGC_PEND` | 是否存在未确认的写入意图 |
| 8 | `NGC_READY` | 基线是否完整建立 |
| 9 | `NGC_BLOCK` | 是否因外部修改而让出控制 |
| 10 | `NGC_OWNER` | 实例编号（多实例互斥） |
| 11 | `NGC_UNKN` | 原始值未知（只收紧） |
| 12 | `NGC_CEIL` | 保守模式下的速度上限 |
| 13 | `NGC_ACK` | 落地人工确认（唯一允许手动修改的字段） |
| 14 | `NGC_MAGIC` | 魔数 602902，用于识别本脚本创建的表 |

> 除 `NGC_ACK` 外，**不要手动修改任何 `NGC_*` 参数**，否则会破坏恢复记录的一致性。

---

## 状态码 / State Codes

状态码写入飞行日志的 `State` 字段，便于复盘：

| 状态码 | 含义 |
| --- | --- |
| 1 | 未限速（使用原始参数） |
| 2 | 0.50 m/s 档 |
| 3 | 0.10 m/s 档 |
| 4 | 高度数据故障，保守档 |
| 5 | 非目标模式（非 AltHold / Loiter） |
| 6 | 让出控制 / 参数异常 |
| 7 | 初始化未完成或状态未知（只监视） |
| 8 | 原始值未知，只收紧 |
| 9 | 已落地，等待 `NGC_ACK` 确认 |

---

## 飞行日志 / Logging

脚本以 5 Hz（节流 200 ms）写入日志消息 `NGSL`：

```
NGSL, Height, Limit, VDown, Mode, State    (格式 fffBB)
```

- `Height`：相对 home 高度（m），无效时写 NaN
- `Limit`：本周期实际生效的下降速度上限（m/s），无效时写 NaN
- `VDown`：NED 下降速度（m/s）
- `Mode`：当前飞行模式编号，未知时写 255
- `State`：上表状态码

地面站文本消息统一以 `NGS: ` 前缀，级别 3 = error，4 = warning，6 = info。

---

## 安全设计 / Safety Design

1. **不接管飞行控制**：只写 `PILOT_SPEED_DN`（或 `PILOT_SPD_DN`），不发送 arm/disarm/电机/模式指令。
2. **原始值未知只收紧**：空中首次加载、记录损坏等情况下 `ceiling` 只会取更小值，绝不放大参数。
3. **外部修改即让出**：检测到参数被外部改动立即停止写入（`NGC_BLOCK = 1`），直到解锁后重建基线。
4. **写前意图 + 写后读回**：每次写原生参数前先记录 `PVAL` / `PEND`，写入后读回校验；中途掉电也能在下次启动识别。
5. **提交式记录**：`NGC_READY` 先清零、逐字段写入、最后置 1；中途断电会被判定为记录不完整并转保守。
6. **异常不退出**：采样与主逻辑全部包 `pcall`，单次异常只导致本周期进入最紧档，脚本继续运行。

---

## 选型建议 / Which Variant

- `near_ground_slowdown.lua`：2 m 以下 0.10 m/s，接地更软，适合机体贵重、场地狭窄的场景。
- `near_ground_slowdown_stage2_0p2.lua`：2 m 以下 0.20 m/s，最后一米更跟手，但接地冲击更大。

---

## 校验信息 / Checksums

上传文件与本地源文件逐字节一致（SHA256）：

| 文件 | SHA256 |
| --- | --- |
| `near_ground_slowdown.lua` | `269588EA9B71B0D729C6241F208C6105557B7DB946D5A235005FACFC1740FDFC` |
| `near_ground_slowdown_stage2_0p2.lua` | `FA6666316678FD5024786CDDB42E9221192464E16BCC2C4629D26B4E5D3DC927` |

---

## 免责声明 / Disclaimer

参数写入类脚本存在风险。首次使用请在空旷场地、拆桨状态下完成地面验证，确认
`NGC_*` 记录与参数还原行为正常后再实飞；使用后果由使用者自行承担。
