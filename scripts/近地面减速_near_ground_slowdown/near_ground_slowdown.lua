-- Copter 4.6.3 / 4.7.0: pilot descent cap with airborne Lua recovery.
-- CONTROL SCRIPT. Height is EKF ABOVE HOME, not terrain clearance.
-- Load ONE copy. NGC_* is internal RAM recovery state; do not edit it except ACK.
-- Unknown-original airborne load only TIGHTENS. After landing restore/check the
-- required native parameter, then set NGC_ACK=1 while disarmed to confirm it.
-- No Lua arm/disarm/motor/mode commands. Native landing + DISARM_DELAY disarm.
-- ===========================================================================
-- 【中文总说明】近地限速脚本（ArduCopter 4.6.3 / 4.7.0）
--   功能：在 AltHold(2) / Loiter(5) 模式下，按“EKF 相对 home 点的高度”自动收紧
--         飞行员手动下降速度参数，防止手动降落时下降过快砸地：
--           高度 > 5 m       -> 使用原始参数（不限制）
--           2 m < 高度 <= 5 m -> 下降速度限制 0.5 m/s
--           高度 <= 2 m      -> 下降速度限制 0.1 m/s
--   重要限制：高度是 EKF 相对 home 点的高，不是对地真实高度；
--             地形起伏 / 气压漂移 / home 点偏差都会影响判定。
--   安全设计：
--     1) 脚本只修改参数，绝不发送 arm/disarm/电机/模式指令；
--        落地靠固件原生 DISARM_DELAY 自动上锁（若该参数为 0 会持续告警）。
--     2) 不知道原始参数值时（空中首次加载、记录损坏等）只允许“收紧”，
--        绝不向上放大参数，状态码 8，需落地人工核对。
--     3) 检测到外部改参数会立刻让出控制，直到解锁状态下重新建立基线。
--     4) 所有对原生参数的写入都有“写前意图记录 + 写后读回校验”。
--   使用流程：飞控上只加载一份；落地后人工恢复/核对原生参数，
--             然后在【未解锁】状态下把 NGC_ACK 置 1 确认新基线。
--   NGC_* 为脚本内部恢复状态参数，除上面的 NGC_ACK 外不要手动修改。
-- ===========================================================================

-- 周期与档位参数：
--   PERIOD       回调周期 100 ms（10 Hz）
--   GAP_MS       两次回调间隔超过 500 ms 视为调度卡顿（数据不可信，走保守档）
--   RELEASE_MS   档位“往松方向”释放需要持续的时长 500 ms（防边界抖动）
--   HYST         释放滞回 0.3 m（掉档立刻、放档慢半拍）
local PERIOD,GAP_MS,RELEASE_MS,HYST=100,500,500,0.3
-- 恢复记录参数表：前缀 'NGC_'、表序号 192、魔数 602902、结构版本 2。
-- 魔数/版本用于识别“这张表是不是本脚本创建的、结构是否匹配”。
local PREFIX,TABLE_KEY,MAGIC,VERSION='NGC_',192,602902,2
-- 恢复表字段（顺序即 param 索引 1..15）：
--   VER 结构版本 / ADAPTER 参数名适配器编号 / BASE 原始下降参数 / UP 原始上升参数
--   EXPECT 脚本认为当前参数应等于的值 / PVAL 写前意图值 / PEND 写前意图是否有未确认项
--   READY 基线是否完整建立 / BLOCK 是否因外部修改而让出控制 / OWNER 实例编号
--   UNKN 原始值是否未知（只收紧） / CEIL 保守模式下的速度上限 / ACK 落地确认 / MAGIC 魔数
local FIELDS={'VER','ADAPTER','BASE','UP','EXPECT','PVAL','PEND','READY',
              'BLOCK','OWNER','UNKN','CEIL','ACK','MAGIC'}

-- 适配器：兼容两代固件参数命名。
--   key     要控制的“下降速度”参数名
--   up_key  对应的“上升速度”参数名（DN=0 时固件会回退到 UP 值）
--   scale   参数值 -> m/s 的除数（4.6 是 cm/s -> 100；4.7 是 m/s -> 1）
--   adapter 记录在 NGC_ADAPTER 里的编号
local key,up_key,scale,adapter

-- 参数控制相关的核心变量：
--   original / original_up  原始下降 / 上升参数值（要还原的目标）
--   effective               原始“有效下降速度”(m/s)，即 DN=0 时取 UP 的效果值
--   expected                脚本认为参数当前应等于的值，用于检测外部修改
--   owned                   脚本自己写出去的、非原始的值（需要还原）；原始值则为 nil
--   pending                 已发出但尚未确认的写入意图值
--   ceiling                 仅收紧模式下的速度上限（记录在 NGC_CEIL）
local original,original_up,effective,expected,owned,pending,ceiling

-- 运行状态标志：
--   initialized  已完成初始化（适配器 + 恢复表）
--   cache        恢复表可用（可持久化状态）；false 表示只能“看着办”
--   prepared     已建立可信基线，可以闭环控制参数
--   blocked      检测到外部改参数，已让出控制直到解锁后重建基线
--   unknown      原始值未知 -> 只允许收紧（tighten only）
--   resuming     正在从 RAM 恢复上下文（需要重新推断当前档位）
local initialized,cache,prepared,blocked,unknown,resuming=false,false,false,false,false,false

-- 记账与限频用时间戳：
--   owner 本实例编号（递增，用于多实例互斥）
--   last_ms 上次回调时刻
--   release_since / recovery_since 档位释放计时 / 故障恢复计时
--   last_log / last_warning / last_notice / last_delay 日志与告警的限频时间
local owner,last_ms,release_since,recovery_since,last_log,last_warning,last_notice,last_delay

-- band    当前档位：0=不限，1=0.5 m/s，2=0.1 m/s
-- fault   高度数据故障中（保守档）
-- state   状态码，写入日志便于复盘
-- confirmed_limit 本周期实际生效的限幅(m/s)，仅用于日志
local band,fault,state,confirmed_limit=0,false,0,nil
-- ACK 必须观察到“新的上升沿”（先看到 0 再看到 1）才生效，
-- 防止沿用上一次落地遗留的 ACK 或 VM 启动瞬间的旧值。
local ack_ready=false -- A fresh ACK edge must be observed after a locked zero.

-- 是否为有限数（排除 nil / NaN / Inf）。
local function finite(x) return type(x)=='number' and x==x and math.abs(x)<math.huge end
-- 浮点近似相等，用于参数读回比较（参数是 float32，不能直接用 ==）。
local function equal(a,b) return finite(a) and finite(b) and math.abs(a-b)<0.00001 end
-- 毫秒计数是 32 位回绕的，这里做回绕安全的差值（a 在 b 之后则结果为小时长）。
local function elapsed(a,b) return (a-b)%4294967296 end
-- 向地面站发文本消息：级别越小越严重（3=error，4=warning，6=info），失败不影响控制。
local function message(level,text) pcall(function() gcs:send_text(level,'NGS: '..text) end) end
-- 限频告警：同一条通道最短间隔 2 s，避免刷屏拖慢脚本调度。
local function warning(now,text)
    if not last_warning or elapsed(now,last_warning)>=2000 then message(4,text); last_warning=now end
end
-- 读飞控参数，参数不存在返回 nil。
local function read(name) return param:get(name) end
-- 读恢复表中的字段（自动加 NGC_ 前缀）。
local function context(name) return read(PREFIX..name) end
-- 写恢复表字段并读回校验；失败（或抛异常）只告警并返回 false，由调用者降级处理。
local function store(now,name,value)
    local ok,result=pcall(function()
        return param:set(PREFIX..name,value)==true and equal(context(name),value)
    end)
    if not ok or not result then warning(now,'parameter write/readback failed (recovery record)'); return false end
    return true
end
-- 本实例是否仍持有控制权：无恢复表时视为持有（单实例场景），
-- 有恢复表时 NGC_OWNER 必须等于自己的 owner，否则说明有更新的实例接管了。
local function ours() return not cache or equal(context('OWNER'),owner) end

-- 初始化恢复表：尽量安全地“创建 / 校验”NGC_* 参数表，并取得本实例编号。
-- 返回 true 表示恢复表可用（cache=true）；false 表示只能用内存状态（保守模式）。
local function init_context(now)
    local signature=context('MAGIC')
    if signature==nil then
        -- 没有魔数，不代表这张表不存在：同前缀的“残缺/外来”记录不能当成空表，
        -- 否则重新 add_table 会把固件元数据和已有值一起重置，且此时还无法判断
        -- 这些值是不是上一个 VM 留下的限幅（可能是非原始值）。
        for _,name in ipairs(FIELDS) do
            if context(name)~=nil then warning(now,'incomplete recovery table; conservative only'); return false end
        end
        -- 真正的空表：注册参数表（table key 192，前缀 NGC_，15 个字段）。
        if not param.add_table or not param:add_table(TABLE_KEY,PREFIX,#FIELDS) then
            warning(now,'recovery table unavailable; conservative only'); return false
        end
        for index,name in ipairs(FIELDS) do
            if not param:add_param(TABLE_KEY,index,name,0) then
                warning(now,'recovery table registration failed'); return false
            end
        end
        -- 这些键在本次固件启动时是不存在的，因此磁盘上残留的旧状态一律无效，
        -- 全部清零后写入版本号和魔数（最后写魔数，作为“表已就绪”的标记）。
        for _,name in ipairs(FIELDS) do if not store(now,name,0) then return false end end
        if not store(now,'VER',VERSION) or not store(now,'MAGIC',MAGIC) then return false end
    elseif not equal(signature,MAGIC) or not equal(context('VER'),VERSION) then
        -- 表存在但魔数 / 结构版本不匹配：结构不同，字段语义不可信，只做保守控制。
        warning(now,'recovery record incompatible; conservative only'); return false
    end
    -- 注意：对已存在的参数绝不能再次 add_param，固件会把它重新加载为默认值。
    -- 这里只读取上一次的 owner 并 +1，形成“新实例编号”，让旧实例自动失效。
    local previous=context('OWNER')
    if not finite(previous) or previous<0 or previous>=1000000 or previous%1~=0 then return false end
    owner=(previous+1)%1000000
    return store(now,'OWNER',owner)
end
-- 选择参数适配器：4.6 用 PILOT_SPEED_DN/UP（cm/s），4.7 用 PILOT_SPD_DN/UP（m/s）。
-- 必须“且仅有一个”存在，否则说明固件不认识这两组参数或同时存在，无法安全判断。
-- （两者同时存在属于异常情况，直接放弃，避免改错参数。）
local function select_adapter(now)
    local old,new=read('PILOT_SPEED_DN'),read('PILOT_SPD_DN')
    if finite(old) and new==nil then key,up_key,scale,adapter='PILOT_SPEED_DN','PILOT_SPEED_UP',100,46
    elseif finite(new) and old==nil then key,up_key,scale,adapter='PILOT_SPD_DN','PILOT_SPD_UP',1,47
    else warning(now,'unsupported descent parameter adapter'); return false end
    return true
end
-- 读取当前“有效下降速度”：返回 (m/s, 原始下降参数值, 原始上升参数值)。
-- 固件语义：DN=0 表示“使用 UP 的值”，所以 DN=0 时用 UP 换算。
-- 参数非法（nil / 负数 / DN=0 且 UP 无效）时返回 nil，调用者据此转保守。
local function current_effective()
    local down,up=read(key),read(up_key)
    if not finite(down) or down<0 or (down==0 and (not finite(up) or up<=0)) then return nil end
    return (down==0 and up or down)/scale,down,up
end
-- 一次性初始化：选适配器 -> 建/校验恢复表 -> 从恢复表还原上下文。
-- 三种结果：
--   A. 恢复表完整有效 -> 直接恢复基线并进入监视/控制（resuming=true，档位待重算）
--   B. 恢复表有效但处于“原始值未知”模式 -> 恢复为“只收紧”模式
--   C. 恢复表“看起来已完成但内容非法” -> 认定上一个 VM 可能留下了限幅，
--      绝不把它当原始值，改为未知模式并要求人工重建基线
local function initialize(now)
    if not select_adapter(now) then return false end
    cache=init_context(now)
    if cache then
        blocked=equal(context('BLOCK'),1); unknown=equal(context('UNKN'),1)
        local base,up,exp,pval,pend=context('BASE'),context('UP'),context('EXPECT'),context('PVAL'),context('PEND')
        -- 只有全部字段自洽（READY=1、适配器一致、数值合法、无未决意图、非未知模式）
        -- 才认为“上一次会话的记录可信”，直接沿用。
        if equal(context('READY'),1) and equal(context('ADAPTER'),adapter)
           and finite(base) and base>=0 and finite(up) and up>0 and finite(exp) and exp>=0
           and (equal(pend,0) or (equal(pend,1) and finite(pval) and pval>=0)) and not unknown then
            original,original_up,expected=base,up,exp
            effective=(base==0 and up or base)/scale   -- DN=0 时按 UP 计算等效速度
            pending=equal(pend,1) and pval or nil       -- 恢复“写了一半”的意图
            owned=not equal(exp,base) and exp or nil    -- EXPECT 与原始值不同说明是自己写的
            prepared,resuming=true,true
            message(6,'RAM context recovered; monitoring active')
        elseif unknown and equal(context('ADAPTER'),adapter) then
            -- 上次已进入“只收紧”模式：只恢复有用的字段，不假设任何原始值。
            expected=context('EXPECT')
            if not finite(expected) or expected<0 then expected=nil end
            pending=equal(context('PEND'),1) and context('PVAL') or nil
            ceiling=context('CEIL')
            if not finite(ceiling) or ceiling<=0 then ceiling=nil end
            resuming=true
        elseif equal(context('READY'),1) then
            -- 记录“标记为已完成”但内容非法：很可能描述的是上一个 VM 留下的限幅值。
            -- 绝不能把这个值当成地面上的新原始值，改为未知模式并要求人工基线。
            unknown=true
            if not store(now,'READY',0) or not store(now,'UNKN',1) then cache=false end
            warning(now,'recovery record invalid; manual baseline required')
        end
    else unknown=true end   -- 没有恢复表（或表不可用）：内存态运行，只收紧
    initialized=true; return true
end
-- 建立基线：把“当前参数值”认定为原始值并写进恢复表（只在未解锁状态调用）。
-- 只有落地后人工 ACK 或解锁状态下的正常运行流程才会走到这里。
-- 写入顺序很重要：先 READY=0（声明记录不完整），再写各字段，最后 READY=1（提交）。
-- 中途掉电 => READY 仍为 0 => 下次启动判定为“记录不完整”，走保守。
local function baseline(now)
    local eff,raw,up=current_effective()
    if not eff or not finite(up) or up<=0 then warning(now,'invalid original descent parameters'); return false end
    if not cache then warning(now,'no recovery record; check parameters manually'); return false end
    -- 已有可信基线且参数没变、也不是未知/让出状态：无需重写（省 EEPROM）。
    if prepared and equal(raw,original) and equal(up,original_up) and not unknown and not blocked then return true end
    if not store(now,'READY',0) then return false end
    for _,item in ipairs({{'ADAPTER',adapter},{'BASE',raw},{'UP',up},{'EXPECT',raw},
                          {'PEND',0},{'BLOCK',0},{'UNKN',0},{'CEIL',eff},{'ACK',0}}) do
        if not store(now,item[1],item[2]) then return false end
    end
    if not store(now,'READY',1) then return false end
    -- 提交成功后同步内存状态：原始值 = 当前值，没有欠还原的写入，档位归零。
    original,original_up,effective,expected=raw,up,eff,raw
    owned,pending=nil,nil
    prepared,blocked,unknown,resuming=true,false,false,false
    band,fault,release_since,recovery_since=0,false,nil,nil
    message(6,string.format('HOME height; %s original=%.3f',key,raw))
    return true
end
-- 进入“原始值未知、只收紧”的保守模式（空中首次加载、记录损坏、读参失败等）。
-- 原则：把上限 ceiling 设为当前有效速度与历史上限中的较小值，只允许继续变小。
local function enter_conservative(now)
    local eff,raw=current_effective()
    if not eff then warning(now,'parameter read failed; monitoring only'); return false end
    local candidate=expected or raw
    ceiling=ceiling and math.min(ceiling,eff) or eff
    unknown,prepared=true,false
    if cache then
        if not store(now,'ADAPTER',adapter) or not store(now,'EXPECT',candidate)
           or not store(now,'CEIL',ceiling) or not store(now,'UNKN',1) then return false end
    end
    -- 只有保守模式元数据全部写成功后才承认 expected：
    -- 这样万一写失败，下次回调会重新初始化，且在第一次收紧前不会误判“外部修改”。
    -- Keep expected nil until the conservative commit is complete: a failed
    -- metadata write must retry initialization before the first native cap.
    expected=candidate
    message(4,'unknown original; tighten only')
    return true
end
-- 发现参数被外部（人或其他脚本）改动：立刻让出控制，直到解锁状态重新建立基线。
local function external(now)
    blocked,prepared,owned,pending=true,false,nil,nil
    if cache and ours() then store(now,'BLOCK',1) end
    state=6; message(3,'external parameter edit; yielding until disarmed')
end
-- 确认一次写入完成：更新 EXPECT 与 owned，并清掉写前意图 PEND。
-- 注意顺序：先写 EXPECT 再清 PEND，这样任一中间快照都能被 reconcile 正确识别。
local function confirm(now,value)
    expected=value; owned=prepared and not equal(value,original) and value or nil
    -- EXPECT commits before PEND clears; either intermediate snapshot is recoverable.
    if cache and (not store(now,'EXPECT',value) or not store(now,'PEND',0)) then return false end
    pending=nil; return true
end
-- 对账：判断“当前参数值”是否仍是脚本预期的值，并处理上次未确认的写入意图。
-- 返回当前参数值；返回 nil 表示读取失败 / 已让出控制（调用者本周期不要再写）。
local function reconcile(now)
    local current=read(key)
    if not finite(current) then warning(now,'parameter read failed'); return nil end
    if pending~=nil then
        if equal(current,pending) then
            -- 上次写入其实成功了（只是没来得及确认），现在补确认。
            if not confirm(now,pending) then return nil end
        elseif equal(current,expected) then
            -- 上次写入没生效，参数还停在预期值：撤销意图。
            if cache and not store(now,'PEND',0) then return nil end
            pending=nil
        end
    end
    -- 既不是预期值也不是待确认意图值 => 被外部改过。
    if not equal(current,expected) then external(now); return nil end
    return current
end
-- 安全写原生下降速度参数：
--   1) 先把意图写进恢复表（PVAL + PEND=1），再改原生参数，最后读回校验；
--   2) 任何一步失败都留有余地（下次回调可凭 PEND 判断到底改没改成）。
local function apply(now,value)
    if not ours() then warning(now,'newer script instance owns control'); return false end
    local current=reconcile(now)
    if current==nil then return false end
    if equal(current,value) then return true end
    -- 写前日志：先记录意图，再改原生参数（这是崩溃可恢复的关键）。
    -- Confirm write-ahead intent before changing the native descent parameter.
    if cache and (not store(now,'PVAL',value) or not store(now,'PEND',1)) then return false end
    pending=value
    local ok=param:set(key,value)
    local after=read(key)
    if equal(after,value) then
        if not confirm(now,value) then return false end
    elseif equal(after,expected) then
        -- 写入没落地，参数仍是原预期值：撤销意图，下次再试。
        if cache and not store(now,'PEND',0) then return false end
        pending=nil
    end
    if ok~=true or not equal(after,value) then warning(now,'parameter write/readback failed'); return false end
    return true
end
-- 还原原始下降参数（只在自己确实改过、或有未决意图时才写）。
local function restore(now)
    if not prepared then return false end
    if owned~=nil or pending~=nil then
        if not apply(now,original) then return false end
        message(6,'original descent parameter restored')
    end
    return true
end
-- 读取解锁状态与飞行模式；任一读取失败返回 nil（调用者按“未知”处理，不动作）。
-- 返回 armed(true/false/nil), mode(数值/nil)。
local function flight_state()
    local a,armed=pcall(function() return arming:is_armed() end)
    local m,mode=pcall(function() return vehicle:get_mode() end)
    if not a then armed=nil end
    if not m then mode=nil end
    return armed,mode
end
-- 采样一次飞行状态：
--   height 相对 home 的 EKF 高度（NED 的 -z）
--   down   下降速度（NED z 向上为正，所以 z>0 表示下降）
--   valid  数据可信条件：EKF 健康 + home 已设置 + 高度为有限数
local function snapshot()
    local armed,mode=flight_state()
    local p=ahrs:get_relative_position_NED_home(); local h=p and -p:z() or nil
    local v=ahrs:get_velocity_NED(); local down=v and v:z() or nil
    return {height=h,valid=ahrs:healthy() and ahrs:home_is_set() and finite(h),
            down=finite(down) and down or nil,armed=armed,mode=mode}
end
-- 主逻辑，每个周期调用一次。
-- 状态机顺序：初始化 -> 解锁状态判断 -> 让出/保守判断 -> 模式过滤
--             -> 对账 -> 高度分档（含滞回）-> 计算并写入目标参数
local function run(now,s)
    -- 回调间隔过长（CPU 卡顿）时数据不可信，本周期强制走保守档。
    local gap=last_ms~=nil and elapsed(now,last_ms)>GAP_MS; last_ms=now
    if not initialized and not initialize(now) then state=7; return end
    if not ours() then state=7; warning(now,'newer script instance owns control'); return end
    -- 若 DISARM_DELAY 为 0/读不到，落地后不会自动上锁，脚本会持续提醒（每 10 s 一次）。
    local delay_ok,delay=pcall(read,'DISARM_DELAY')
    if not delay_ok or not finite(delay) or delay<=0 then
        if not last_delay or elapsed(now,last_delay)>=10000 then
            message(4,'DISARM_DELAY disabled/unknown; native auto-disarm unavailable'); last_delay=now
        end
    end
    -- ===== 未解锁（落地/地面）处理 =====
    if s.armed==false then
        if unknown then
            -- 原始值未知：不能自己决定“原始值”，必须等人工在地面确认。
            state=9
            local ack=cache and context('ACK') or nil
            if equal(ack,0) then ack_ready=true end   -- 先看到 0，才承认后续的 1 是“新按下的”
            if equal(ack,1) and ack_ready then
                if baseline(now) then ack_ready=false end
            else
                -- 不接受“解锁期间留下的 ACK”，也不接受 VM 启动时就存在的 ACK；
                -- 即使复位 ACK 失败、后面又在地面重启导致残留，也仍然安全（先 0 后 1 才算）。
                -- Never accept an ACK left armed, or present at VM startup.
                -- Even a failed reset followed by a disarmed restart stays safe.
                if equal(ack,1) then store(now,'ACK',0) end
                warning(now,cache and 'landed: restore/check, then NGC_ACK=1'
                    or 'landed: check native parameter; recovery table unavailable')
            end
        elseif restore(now) or not prepared then
            -- 正常路径：先把参数还原成原始值，再（重新）建立基线。
            baseline(now)
        end
        last_notice=nil; return
    end
    -- ===== 解锁状态未知（读不到 arming 状态）：只监视，不动参数 =====
    if s.armed~=true then
        ack_ready=false
        release_since,recovery_since=nil,nil
        state=7; warning(now,'arming state unknown; monitoring only'); return
    end
    ack_ready=false
    -- 解锁期间写 ACK 无效：清零并提示（防止在飞行中被误认为已确认基线）。
    if cache and equal(context('ACK'),1) then store(now,'ACK',0); warning(now,'NGC_ACK ignored while armed') end
    -- 已让出控制：必须落地后由 baseline 重新接管。
    if blocked then state=6; warning(now,'external edit; waiting for disarmed baseline'); return end
    -- 没有基线且不知道 expected：进入“只收紧”模式（必须先于任何写入）。
    if not prepared and expected==nil then if not enter_conservative(now) then state=7; return end end
    if not prepared and not unknown then if not enter_conservative(now) then state=7; return end end
    -- 模式值非法（读不到 / 越界）：保持当前参数不动（宁可不限速也不乱写）。
    if not finite(s.mode) or s.mode<0 or s.mode>255 or s.mode%1~=0 then
        release_since,recovery_since=nil,nil
        state=7; warning(now,'mode unknown; keeping current parameter'); return
    end
    -- 只接管 AltHold(2) 与 Loiter(5)；其它模式（Auto/Land/RTL 等）交还固件原生逻辑。
    if s.mode~=2 and s.mode~=5 then
        if prepared and restore(now) then band,fault,release_since,recovery_since=0,false,nil,nil end
        state=5; last_notice=nil; return
    end
    -- 对账（只读）：确认参数没有被外部改动，并处理上次未确认的写入意图。
    -- 注意这里绝不写入过期的 EXPECT（否则会在错误时机把参数改回去）。
    local current=reconcile(now) -- Read-only: never write an obsolete EXPECT here.
    if current==nil then state=6; return end
    -- 特例：原始下降参数为 0（表示“使用 UP 值”）时，UP 也是基线的一部分。
    local fallback_invalid=false
    if prepared and original==0 then
        local up=read(up_key); fallback_invalid=not finite(up) or up<=0
        if not fallback_invalid and not equal(up,original_up) then
            -- 有人改了 UP：先尝试还原自己改过的值，然后让出控制。
            if restore(now) then external(now) end; return
        end
    end
    local current_eff=current_effective()
    if not current_eff and fallback_invalid then current_eff=effective end
    if not current_eff then state=6; warning(now,'parameter read failed'); return end
    -- 从 RAM 恢复后，不知道当前档位：按“当前速度相对基准的速度”反推档位，
    -- 保证不会因为误判档位而把参数放大（宁可判成更紧的档）。
    if resuming then
        local reference=prepared and effective or current_eff
        band=current_eff<=math.min(reference,.1)+.00001 and 2
            or current_eff<=math.min(reference,.5)+.00001 and 1 or 0
        resuming=false
    end
    -- 数据不可信（EKF 异常 / home 未设 / 调度卡顿 / DN=0 但 UP 无效）：强制最保守档。
    if not s.valid or gap or fallback_invalid then
        fault,band,recovery_since,release_since=true,2,nil,nil
        warning(now,'height/gap invalid; conservative descent cap')
    elseif fault then
        -- 故障恢复需要持续 0.5 s 好数据，避免在故障边缘反复放宽。
        recovery_since=recovery_since or now
        if elapsed(now,recovery_since)>=RELEASE_MS then fault,recovery_since=false,nil end
    end
    -- 高度分档：收紧立即生效（tighter），放松要满足滞回并持续 RELEASE_MS。
    if s.valid and not fault then
        local tighter=s.height<=2 and 2 or s.height<=5 and 1 or 0
        if tighter>band then band=tighter; release_since=nil
        else
            local release=(band==2 and s.height>2+HYST) or (band==1 and s.height>5+HYST)
            if release then
                release_since=release_since or now
                if elapsed(now,release_since)>=RELEASE_MS then band=s.height<=5+HYST and 1 or 0; release_since=nil end
            else release_since=nil end
        end
    end
    -- base：已知原始值时以原始有效速度为准；未知时取“历史上限”与当前值的较小者（只收紧）。
    local base=prepared and effective or math.min(ceiling or current_eff,current_eff)
    -- 本周期最终允许的下降速度：0 档不限，1 档 0.5，2 档 0.1（且不超过 base）。
    local limit=band==2 and math.min(base,.1) or band==1 and math.min(base,.5) or base
    local target
    if prepared then
        -- 已建立基线：0 档还原原始参数（保持飞手原设定），其它档写限幅换算后的参数值。
        target=band==0 and original or limit*scale
    else
        -- 未知模式：只有在“确实更紧”时才动参数，绝不放大。
        ceiling=math.min(base,limit)
        if cache and not store(now,'CEIL',ceiling) then state=6; return end
        target=limit<current_eff-.00001 and limit*scale or current
    end
    local ok=apply(now,target)
    -- 状态码（同时写入日志）：
    --   1=未限速 2=0.5 档 3=0.1 档 4=数据故障保守档 5=非目标模式
    --   6=让出/异常 7=初始化未完成 8=原始值未知(只收紧) 9=落地待 ACK
    state=not ok and 6 or unknown and 8 or fault and 4 or band==2 and 3 or band==1 and 2 or 1
    if ok then
        confirmed_limit=limit
        -- 限幅变化时才提示，避免刷屏。
        local notice=string.format('limit %.2f m/s%s',limit,unknown and ' (tighten only)' or fault and ' (height fault)' or '')
        if notice~=last_notice then message(6,notice); last_notice=notice end
    end
end
-- 写飞行日志（5 Hz，节流 200 ms）：高度、当前限幅、下降速度、模式、状态码。
-- 用 0/0（NaN）表示“本次没有有效值”，便于日志里画断点。
local function log_sample(now,s)
    if last_log and elapsed(now,last_log)<200 then return end
    last_log=now
    pcall(function() logger:write('NGSL','Height,Limit,VDown,Mode,State','fffBB',
        s and s.valid and s.height or 0/0,confirmed_limit or 0/0,
        s and s.down or 0/0,s and finite(s.mode) and s.mode or 255,state) end)
end
-- 脚本入口回调：采样 -> 主逻辑 -> 写日志，返回下一次调用间隔。
-- 所有外部读取都包 pcall：任何一处抛异常都不会让脚本整体退出。
local update
update=function()
    local now=millis():tofloat(); confirmed_limit=nil
    local sampled,s=pcall(snapshot)
    if not sampled then
        -- 采样失败：仍要拿到解锁/模式信息，并强制本周期保守处理。
        local armed,mode=flight_state()
        s={valid=false,armed=armed,mode=mode}
        warning(now,'height sample unavailable')
    end
    local ok,err=pcall(run,now,s)
    if not ok then
        -- 主逻辑异常：立即进入最保守档并记录原因（不重启脚本，下一周期继续）。
        fault,band,recovery_since,release_since=true,2,nil,nil
        state=6; warning(now,'runtime failed: '..tostring(err))
        -- 下一次回调会先对账写前意图（PEND），然后再进行任何新写入。
        -- Next callback reconciles the write-ahead intent before any new write.
    end
    log_sample(now,s)
    return update,PERIOD
end
return update,PERIOD
