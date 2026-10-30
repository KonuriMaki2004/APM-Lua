-- Variant: <=5m 0.50 m/s; <=2m 0.20 m/s; fault cap 0.10 m/s.
-- Copter 4.6.3 / 4.7.0: pilot descent cap with airborne Lua recovery.
-- CONTROL SCRIPT. Height is EKF ABOVE HOME, not terrain clearance.
-- Load ONE copy. NGC_* is internal RAM recovery state; do not edit it except ACK.
-- Unknown-original airborne load only TIGHTENS. After landing restore/check the
-- required native parameter, then set NGC_ACK=1 while disarmed to confirm it.
-- No Lua arm/disarm/motor/mode commands. Native landing + DISARM_DELAY disarm.
local PERIOD,GAP_MS,RELEASE_MS,HYST=100,500,500,0.3
local PREFIX,TABLE_KEY,MAGIC,VERSION='NGC_',192,602902,2
local FIELDS={'VER','ADAPTER','BASE','UP','EXPECT','PVAL','PEND','READY',
              'BLOCK','OWNER','UNKN','CEIL','ACK','MAGIC'}
local key,up_key,scale,adapter
local original,original_up,effective,expected,owned,pending,ceiling
local initialized,cache,prepared,blocked,unknown,resuming=false,false,false,false,false,false
local owner,last_ms,release_since,recovery_since,last_log,last_warning,last_notice,last_delay
local band,fault,state,confirmed_limit=0,false,0,nil
local ack_ready=false -- A fresh ACK edge must be observed after a locked zero.

local function finite(x) return type(x)=='number' and x==x and math.abs(x)<math.huge end
local function equal(a,b) return finite(a) and finite(b) and math.abs(a-b)<0.00001 end
local function elapsed(a,b) return (a-b)%4294967296 end
local function message(level,text) pcall(function() gcs:send_text(level,'NGS: '..text) end) end
local function warning(now,text)
    if not last_warning or elapsed(now,last_warning)>=2000 then message(4,text); last_warning=now end
end
local function read(name) return param:get(name) end
local function context(name) return read(PREFIX..name) end
local function store(now,name,value)
    local ok,result=pcall(function()
        return param:set(PREFIX..name,value)==true and equal(context(name),value)
    end)
    if not ok or not result then warning(now,'parameter write/readback failed (recovery record)'); return false end
    return true
end
local function ours() return not cache or equal(context('OWNER'),owner) end

local function init_context(now)
    local signature=context('MAGIC')
    if signature==nil then
        -- Partial/foreign same-prefix records are not an unused table. Re-adding
        -- would reset their firmware metadata and values before identity is known.
        for _,name in ipairs(FIELDS) do
            if context(name)~=nil then warning(now,'incomplete recovery table; conservative only'); return false end
        end
        if not param.add_table or not param:add_table(TABLE_KEY,PREFIX,#FIELDS) then
            warning(now,'recovery table unavailable; conservative only'); return false
        end
        for index,name in ipairs(FIELDS) do
            if not param:add_param(TABLE_KEY,index,name,0) then
                warning(now,'recovery table registration failed'); return false
            end
        end
        -- These keys were absent in THIS firmware boot: ignore persisted old state.
        for _,name in ipairs(FIELDS) do if not store(now,name,0) then return false end end
        if not store(now,'VER',VERSION) or not store(now,'MAGIC',MAGIC) then return false end
    elseif not equal(signature,MAGIC) or not equal(context('VER'),VERSION) then
        warning(now,'recovery record incompatible; conservative only'); return false
    end
    -- Do not re-add existing parameters: firmware add_param reloads defaults.
    local previous=context('OWNER')
    if not finite(previous) or previous<0 or previous>=1000000 or previous%1~=0 then return false end
    owner=(previous+1)%1000000
    return store(now,'OWNER',owner)
end
local function select_adapter(now)
    local old,new=read('PILOT_SPEED_DN'),read('PILOT_SPD_DN')
    if finite(old) and new==nil then key,up_key,scale,adapter='PILOT_SPEED_DN','PILOT_SPEED_UP',100,46
    elseif finite(new) and old==nil then key,up_key,scale,adapter='PILOT_SPD_DN','PILOT_SPD_UP',1,47
    else warning(now,'unsupported descent parameter adapter'); return false end
    return true
end
local function current_effective()
    local down,up=read(key),read(up_key)
    if not finite(down) or down<0 or (down==0 and (not finite(up) or up<=0)) then return nil end
    return (down==0 and up or down)/scale,down,up
end
local function initialize(now)
    if not select_adapter(now) then return false end
    cache=init_context(now)
    if cache then
        blocked=equal(context('BLOCK'),1); unknown=equal(context('UNKN'),1)
        local base,up,exp,pval,pend=context('BASE'),context('UP'),context('EXPECT'),context('PVAL'),context('PEND')
        if equal(context('READY'),1) and equal(context('ADAPTER'),adapter)
           and finite(base) and base>=0 and finite(up) and up>0 and finite(exp) and exp>=0
           and (equal(pend,0) or (equal(pend,1) and finite(pval) and pval>=0)) and not unknown then
            original,original_up,expected=base,up,exp
            effective=(base==0 and up or base)/scale
            pending=equal(pend,1) and pval or nil
            owned=not equal(exp,base) and exp or nil
            prepared,resuming=true,true
            message(6,'RAM context recovered; monitoring active')
        elseif unknown and equal(context('ADAPTER'),adapter) then
            expected=context('EXPECT')
            if not finite(expected) or expected<0 then expected=nil end
            pending=equal(context('PEND'),1) and context('PVAL') or nil
            ceiling=context('CEIL')
            if not finite(ceiling) or ceiling<=0 then ceiling=nil end
            resuming=true
        elseif equal(context('READY'),1) then
            -- An existing but invalid completed record may describe a cap left
            -- by the previous VM. Never bless that cap as a fresh ground original.
            unknown=true
            if not store(now,'READY',0) or not store(now,'UNKN',1) then cache=false end
            warning(now,'recovery record invalid; manual baseline required')
        end
    else unknown=true end
    initialized=true; return true
end
local function baseline(now)
    local eff,raw,up=current_effective()
    if not eff or not finite(up) or up<=0 then warning(now,'invalid original descent parameters'); return false end
    if not cache then warning(now,'no recovery record; check parameters manually'); return false end
    if prepared and equal(raw,original) and equal(up,original_up) and not unknown and not blocked then return true end
    if not store(now,'READY',0) then return false end
    for _,item in ipairs({{'ADAPTER',adapter},{'BASE',raw},{'UP',up},{'EXPECT',raw},
                          {'PEND',0},{'BLOCK',0},{'UNKN',0},{'CEIL',eff},{'ACK',0}}) do
        if not store(now,item[1],item[2]) then return false end
    end
    if not store(now,'READY',1) then return false end
    original,original_up,effective,expected=raw,up,eff,raw
    owned,pending=nil,nil
    prepared,blocked,unknown,resuming=true,false,false,false
    band,fault,release_since,recovery_since=0,false,nil,nil
    message(6,string.format('HOME height; %s original=%.3f',key,raw))
    return true
end
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
    -- Keep expected nil until the conservative commit is complete: a failed
    -- metadata write must retry initialization before the first native cap.
    expected=candidate
    message(4,'unknown original; tighten only')
    return true
end
local function external(now)
    blocked,prepared,owned,pending=true,false,nil,nil
    if cache and ours() then store(now,'BLOCK',1) end
    state=6; message(3,'external parameter edit; yielding until disarmed')
end
local function confirm(now,value)
    expected=value; owned=prepared and not equal(value,original) and value or nil
    -- EXPECT commits before PEND clears; either intermediate snapshot is recoverable.
    if cache and (not store(now,'EXPECT',value) or not store(now,'PEND',0)) then return false end
    pending=nil; return true
end
local function reconcile(now)
    local current=read(key)
    if not finite(current) then warning(now,'parameter read failed'); return nil end
    if pending~=nil then
        if equal(current,pending) then if not confirm(now,pending) then return nil end
        elseif equal(current,expected) then
            if cache and not store(now,'PEND',0) then return nil end
            pending=nil
        end
    end
    if not equal(current,expected) then external(now); return nil end
    return current
end
local function apply(now,value)
    if not ours() then warning(now,'newer script instance owns control'); return false end
    local current=reconcile(now)
    if current==nil then return false end
    if equal(current,value) then return true end
    -- Confirm write-ahead intent before changing the native descent parameter.
    if cache and (not store(now,'PVAL',value) or not store(now,'PEND',1)) then return false end
    pending=value
    local ok=param:set(key,value)
    local after=read(key)
    if equal(after,value) then if not confirm(now,value) then return false end
    elseif equal(after,expected) then
        if cache and not store(now,'PEND',0) then return false end
        pending=nil
    end
    if ok~=true or not equal(after,value) then warning(now,'parameter write/readback failed'); return false end
    return true
end
local function restore(now)
    if not prepared then return false end
    if owned~=nil or pending~=nil then
        if not apply(now,original) then return false end
        message(6,'original descent parameter restored')
    end
    return true
end
local function flight_state()
    local a,armed=pcall(function() return arming:is_armed() end)
    local m,mode=pcall(function() return vehicle:get_mode() end)
    if not a then armed=nil end
    if not m then mode=nil end
    return armed,mode
end
local function snapshot()
    local armed,mode=flight_state()
    local p=ahrs:get_relative_position_NED_home(); local h=p and -p:z() or nil
    local v=ahrs:get_velocity_NED(); local down=v and v:z() or nil
    return {height=h,valid=ahrs:healthy() and ahrs:home_is_set() and finite(h),
            down=finite(down) and down or nil,armed=armed,mode=mode}
end
local function run(now,s)
    local gap=last_ms~=nil and elapsed(now,last_ms)>GAP_MS; last_ms=now
    if not initialized and not initialize(now) then state=7; return end
    if not ours() then state=7; warning(now,'newer script instance owns control'); return end
    local delay_ok,delay=pcall(read,'DISARM_DELAY')
    if not delay_ok or not finite(delay) or delay<=0 then
        if not last_delay or elapsed(now,last_delay)>=10000 then
            message(4,'DISARM_DELAY disabled/unknown; native auto-disarm unavailable'); last_delay=now
        end
    end
    if s.armed==false then
        if unknown then
            state=9
            local ack=cache and context('ACK') or nil
            if equal(ack,0) then ack_ready=true end
            if equal(ack,1) and ack_ready then
                if baseline(now) then ack_ready=false end
            else
                -- Never accept an ACK left armed, or present at VM startup.
                -- Even a failed reset followed by a disarmed restart stays safe.
                if equal(ack,1) then store(now,'ACK',0) end
                warning(now,cache and 'landed: restore/check, then NGC_ACK=1'
                    or 'landed: check native parameter; recovery table unavailable')
            end
        elseif restore(now) or not prepared then baseline(now) end
        last_notice=nil; return
    end
    if s.armed~=true then
        ack_ready=false
        release_since,recovery_since=nil,nil
        state=7; warning(now,'arming state unknown; monitoring only'); return
    end
    ack_ready=false
    if cache and equal(context('ACK'),1) then store(now,'ACK',0); warning(now,'NGC_ACK ignored while armed') end
    if blocked then state=6; warning(now,'external edit; waiting for disarmed baseline'); return end
    if not prepared and expected==nil then if not enter_conservative(now) then state=7; return end end
    if not prepared and not unknown then if not enter_conservative(now) then state=7; return end end
    if not finite(s.mode) or s.mode<0 or s.mode>255 or s.mode%1~=0 then
        release_since,recovery_since=nil,nil
        state=7; warning(now,'mode unknown; keeping current parameter'); return
    end
    if s.mode~=2 and s.mode~=5 then
        if prepared and restore(now) then band,fault,release_since,recovery_since=0,false,nil,nil end
        state=5; last_notice=nil; return
    end
    local current=reconcile(now) -- Read-only: never write an obsolete EXPECT here.
    if current==nil then state=6; return end
    local fallback_invalid=false
    if prepared and original==0 then
        local up=read(up_key); fallback_invalid=not finite(up) or up<=0
        if not fallback_invalid and not equal(up,original_up) then if restore(now) then external(now) end; return end
    end
    local current_eff=current_effective()
    if not current_eff and fallback_invalid then current_eff=effective end
    if not current_eff then state=6; warning(now,'parameter read failed'); return end
    if resuming then
        local reference=prepared and effective or current_eff
        band=current_eff<=math.min(reference,.2)+.00001 and 2
            or current_eff<=math.min(reference,.5)+.00001 and 1 or 0
        resuming=false
    end
    if not s.valid or gap or fallback_invalid then
        fault,band,recovery_since,release_since=true,2,nil,nil
        warning(now,'height/gap invalid; conservative descent cap')
    elseif fault then
        recovery_since=recovery_since or now
        if elapsed(now,recovery_since)>=RELEASE_MS then fault,recovery_since=false,nil end
    end
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
    local base=prepared and effective or math.min(ceiling or current_eff,current_eff)
    local limit=fault and math.min(base,.1)
        or band==2 and math.min(base,.2)
        or band==1 and math.min(base,.5) or base
    local target
    if prepared then target=band==0 and original or limit*scale
    else
        ceiling=math.min(base,limit)
        if cache and not store(now,'CEIL',ceiling) then state=6; return end
        target=limit<current_eff-.00001 and limit*scale or current
    end
    local ok=apply(now,target)
    state=not ok and 6 or unknown and 8 or fault and 4 or band==2 and 3 or band==1 and 2 or 1
    if ok then
        confirmed_limit=limit
        local notice=string.format('limit %.2f m/s%s',limit,unknown and ' (tighten only)' or fault and ' (height fault)' or '')
        if notice~=last_notice then message(6,notice); last_notice=notice end
    end
end
local function log_sample(now,s)
    if last_log and elapsed(now,last_log)<200 then return end
    last_log=now
    pcall(function() logger:write('NGSL','Height,Limit,VDown,Mode,State','fffBB',
        s and s.valid and s.height or 0/0,confirmed_limit or 0/0,
        s and s.down or 0/0,s and finite(s.mode) and s.mode or 255,state) end)
end
local update
update=function()
    local now=millis():tofloat(); confirmed_limit=nil
    local sampled,s=pcall(snapshot)
    if not sampled then
        local armed,mode=flight_state()
        s={valid=false,armed=armed,mode=mode}
        warning(now,'height sample unavailable')
    end
    local ok,err=pcall(run,now,s)
    if not ok then
        fault,band,recovery_since,release_since=true,2,nil,nil
        state=6; warning(now,'runtime failed: '..tostring(err))
        -- Next callback reconciles the write-ahead intent before any new write.
    end
    log_sample(now,s)
    return update,PERIOD
end
return update,PERIOD
