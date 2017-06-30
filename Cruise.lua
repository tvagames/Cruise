ACTIVE_MIN_ALT = -10
ACTIVE_MAX_ALT = 500

--------------------
-- 2点間の距離
--------------------
function GetDistance(ax, az, bx, bz)
  return math.sqrt(Mathf.Abs(ax - bx)^2 + Mathf.Abs(az - bz)^2)
end


function GetTablelength(T)
  local count = 0
  for _ in pairs(T) do count = count + 1 end
  return count
end

--------------------
-- 立ち入り禁止の判定
--------------------
function IsForbidden(node)
  return ACTIVE_MIN_ALT < node.y or node.y > ACTIVE_MAX_ALT
end

--------------------
-- ノードを追加
--------------------
function AddServeyNode(I, nodes, x, z, goal, parent)
  key = x .. "_" .. z
  if nodes[key] ~= nil then
    return
  end

  c = 0
  if parent ~= nil then
    c = parent.c + GetDistance(x, z, parent.x, parent.z)
  end

  local node = {
    x = x, 
    y = I:GetTerrainAltitudeForPosition(x, 0, z), 
    z = z,
    c = c, h = GetDistance(goal.x, goal.z, x, z),
    parent = parent,
    status = 0, -- open
    }
  node.score = node.c + node.h

  if IsForbidden(node) then
    node.status = 2 -- forbidden
  end

  nodes[key] = node
  return nodes[key]
end

--------------------
-- ノードを開く
--------------------
function OpenSurveyNode(I, nodes, node, goal, scale)
  local x = node.x
  local z = node.z
  AddServeyNode(I, nodes, x - scale, z - scale, goal, node)
  AddServeyNode(I, nodes, x, z - scale, goal, node)
  AddServeyNode(I, nodes, x + scale, z - scale, goal, node)
  AddServeyNode(I, nodes, x - scale, z, goal, node)
  AddServeyNode(I, nodes, x + scale, z, goal, node)
  AddServeyNode(I, nodes, x - scale, z + scale, goal, node)
  AddServeyNode(I, nodes, x, z + scale, goal, node)
  AddServeyNode(I, nodes, x + scale, z + scale, goal, node)

  node.status = 1 -- close
end

--------------------
-- 経路検索
--------------------
function FindPath(I, start, goal, scale)
  local nodes = {}
  node = AddServeyNode(I, nodes, start.x, start.z, goal, nil)

  while GetDistance(node.x, node.z, goal.x, goal.z) > scale / 2 do
    OpenSurveyNode(I, nodes, node, goal, scale)
    local n = nil
    for key, value in pairs(nodes) do
      if value.status == 0 and (n == nil
      or (value.score < node.score)
      or (value.score == node.score and value.h < node.h)) then
        n = value
      end
    end

    if n == nil then
      break
    else
      node = n
    end
  end

  local waypoints = {}
  while node.parent ~= nil do
    table.insert(waypoints, 1, node)
    node = node.parent
  end
  return waypoints
end

-- =================================================
-- AutoCruiseGuide
-- =================================================
CAN_DETAIL = false
--------------------
-- コンストラクタ
--------------------
function CreateAutoCruiseGuide(I, currentPos, goal)
  local self = {} -- setmetatable({}, AutoCruiseGuide)
  self.goal = goal
  self.roughWaypoints = {}
  self.roughWaypointCount = 0
  self.roughScale = 0
  self.detailGoal = {}
  self.detailWaypoints = {}
  self.detailWaypointCount = 0
  self.detailScale = 0
  d = GetDistance(currentPos.x, currentPos.z, goal.x, goal.z) / 100
  self.roughScale = 300 -- Mathf.Max(1000, d)
    
  self.roughWaypoints = FindPath(I, currentPos, goal, self.roughScale)
  self.roughWaypointCount = table.maxn(self.roughWaypoints)
  
  if CAN_DETAIL == false then
    return self
  end

  if self.roughWaypointCount == 0 then
    I:Log("経路不明")
    return self
  elseif self.roughWaypointCount == 1 then
    self.detailGoal = table.remove(self.roughWaypoints, 1)
  elseif self.roughWaypointCount > 1 then
    table.remove(self.roughWaypoints, 1)
    self.detailGoal = table.remove(self.roughWaypoints, 1)
  end

  self.detailScale = 50 -- Mathf.Max(50, GetDistance(currentPos.x, currentPos.z, self.detailGoal.x, self.detailGoal.z) / 100)
  self.detailWaypoints = FindPath(I, currentPos, self.detailGoal, self.detailScale)
  self.detailWaypointCount = table.maxn(self.detailGoal)
  return self
end

--------------------
-- 到達判定
--------------------
function IsReachedWaypoint(waypoint, pos, scale)
  local d = GetDistance(waypoint.x, waypoint.z, pos.x, pos.z)
  return d < scale
end

--------------------
-- 目的地
--------------------
function GetCurrentWaypoint(self, I)
  local currentPos = I:GetConstructPosition()
  I:Log(self.roughScale)
  if IsReachedWaypoint(self.goal, currentPos, self.roughScale) then
    return nil
  end
  if CAN_DETAIL == false then
    if IsReachedWaypoint(self.roughWaypoints[1], currentPos, self.roughScale) then
      table.remove(self.roughWaypoints, 1)
    end
    return self.roughWaypoints[1]
  end
  
  local dest = self.detailWaypoints[1]
  if IsReachedWaypoint(dest, currentPos, self.detailScale) then
    table.remove(self.detailWaypoints, 1)
  end

  local actualDetailWaypointCount = table.maxn(self.detailWaypoints)
  if actualDetailWaypointCount < self.detailWaypointCount / 2 then
    -- in progress over 50 %
  elseif actualDetailWaypointCount > 0 then
    return self.detailWaypoints[1]
  end

  local actualroughWaypointCount = table.maxn(self.roughWaypoints)
  if actualroughWaypointCount > 1 then
    -- 一歩先
    table.remove(self.roughWaypoints, 1)
  end

  if actualroughWaypointCount > 0 then
    -- 二歩先
    table.remove(self. roughWaypoints, 1)
    self.detailGoal = self.roughWaypoints[1]
    self.detailScale = Mathf.Max(300, GetDistance(currentPos.x, currentPos.z, self.detailGoal.x, self.detailGoal.z) / 100)
    self.detailWaypoints = FindPath(currentPos, self.detailGoal, self.detailScale)
    self.detailWaypointCount = table.maxn(self.detailGoal)
    
    return self.detailWaypoints[1]
  else
    return nil
  end
end



AROUND_DISTANCE = 100

--------------------
-- 二つのベクトルのなす角
--------------------
function Angle(a, b)
  return math.deg(math.acos(Vector3.Dot(a, b)
 / (Vector3.Magnitude(a) * Vector3.Magnitude(b))))
end

--------------------
-- ヨー
-- YawLeft = 0,YawRight = 1,
-- RollLeft = 2,RollRight = 3,
-- NoseUp = 4,NoseDown = 5,
-- Increase = 6,Decrease = 7,
-- MainPropulsion = 8
--------------------
function ControlYaw(I, drive, isRight)
  if isRight then
    I:RequestControl(0,1,drive)
  else
    I:RequestControl(0,0,drive)
  end
end

--------------------
-- 前へ
--------------------
function Forward(I, drive)
  I:RequestControl(0,8,drive)
end



------------------------------------------
-- PID
------------------------------------------
ypid = {
  logs = {}, kP = 0.5, Ti = 200, Td = 5, 
  total = 0, counter = 0, score = 0, phase = 1,
  best = {score = -1, kP = 0.49, Ti = 200, Td = 3},
  default = {score = -1, kP = 0.49, Ti = 200, Td = 3}
  }

---------------------
-- PID
---------------------
function GetGain(I, name, obj, target, actual)
    value = target - actual
    --obj = selfTuning(I, name, obj, value)

    -- P動作
    p = value * obj.kP

    -- I動作
    obj.total = obj.total + value
    table.insert(obj.logs, 1, value)
    c = table.maxn(obj.logs)

    while c > obj.Ti do
        obj.total = obj.total - obj.logs[c]
        table.remove(obj.logs, c)
        c = table.maxn(obj.logs)
    end  
    i = (obj.total / obj.Ti) * obj.kP

    -- D動作
    if c > obj.Td then
      diff = obj.logs[obj.Td] - value
    else
      diff = obj.logs[c] - value
    end
    d = diff * obj.kP

    return p + i + d
end


----------------------------------------
-- 航行
----------------------------------------
turnDirectionLock = "" -- 振り向く方向スイッチ
aroundDirectionLock = "" -- 周回方向スイッチ

--------------------
-- 周回ポジションの取得
--------------------
function GetAroundPosition(I, selfPostion, targetPosition)
    -- 目標から見たベクトル
    vn = (selfPostion - targetPosition).normalized

    -- 周回方向の決定
    if aroundDirectionLock == "" then
      side = Vector3.Dot((targetPosition - selfPostion).normalized, I:GetConstructRightVector())
      if side > 0 then
        turnDirectionLock = "right"
      else
        turnDirectionLock = "left"
      end
    end

    if aroundDirectionLock == "right" then
      p = Quaternion.Euler(0, 45, 0) * vn
    else
      p = Quaternion.Euler(0, -45, 0) * vn
    end
    return targetPosition + (p * AROUND_DISTANCE)
end

---------------------
-- 行先の決定
---------------------
function GetWaypoint(I)
  r = I:GetFriendlyInfo(0)
  return r.CenterOfMass
end

---------------------
-- 目的地に向かって巡行
---------------------
function Cruise(I, destination)
  pos = I:GetConstructPosition()
  t = (destination - pos).normalized
  fvec = I:GetConstructForwardVector()
  side = Vector3.Dot(t, I:GetConstructRightVector())
  front = Vector3.Dot(t, I:GetConstructForwardVector())

  angle = Angle(fvec, t)

  drive = 1

  gain = Mathf.Abs(GetGain(I, "", ypid, 0, angle))
  if front > 0 then
    I:Log("angle" .. angle .. ", side" .. side .. ", front" .. front .. ", gain" .. gain)
    ControlYaw(I, gain, (side > 0))
    turnDirectionLock = ""

  else
    -- 振り向く方向の決定
    if turnDirectionLock == "" then
      if side > 0 then
        turnDirectionLock = "right"
      else
        turnDirectionLock = "left"
      end
    end

    ControlYaw(I, drive, (turnDirectionLock == "right"))
  end

  Forward(I, drive)

end

guide = nil
---------------------
-- FromTheDepths
---------------------
function Update(I)
  I:ClearLogs()
  pos = I:GetConstructPosition()
  course = GetWaypoint(I)
  if course ~= nil then
    if guide == nil then
      guide = CreateAutoCruiseGuide(I, pos, course)
    end
    w = GetCurrentWaypoint(guide, I)
    if w == nil then
      guide = nil
    else
      c = Vector3(w.x, w.y, w.z)
      p = GetAroundPosition(I, pos, c)
      Cruise(I, p)
    end
  end
end
