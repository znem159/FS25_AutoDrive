MGPF = {}

MGPF.MAX_STEPS = 10000
MGPF.WIDTH_ADDITON = 1.35
MGPF.RADIUS_ADDITION = 1.35
MGPF.WIDTH_SCALER = 0.05

function MGPF:new(vehicle)
    local o = {}
	setmetatable(o, self)
    self.__index = self

    o.mask = CollisionFlag.DEFAULT + CollisionFlag.STATIC_OBJECT + CollisionFlag.DYNAMIC_OBJECT + CollisionFlag.VEHICLE + CollisionFlag.TERRAIN_DELTA + CollisionFlag.TREE + CollisionFlag.BUILDING
    o.stepSize = 1.5
    o.groundClearance = 0.5
    o.vehicle = vehicle
    
    return o
end

function MGPF:startPathPlanningToNetwork(destinationId)
    local closest = self.vehicle:getClosestWayPoint()
    self.goingToNetwork = true
    return self:startPathPlanningToWayPoint(closest, destinationId)
end

function MGPF:startPathPlanningToWayPoint(wayPointId, destinationId)
    local targetNode = ADGraphManager:getWayPointById(wayPointId)
    local wayPoints = ADGraphManager:pathFromTo(wayPointId, destinationId)
    self.resultPath = nil

    if wayPoints ~= nil and #wayPoints > 1 then
        local vecToNextPoint = {x = wayPoints[2].x - targetNode.x, z = wayPoints[2].z - targetNode.z}
        self.goingToNetwork = true
        self.destinationId = destinationId
        self.targetWayPointId = wayPointId
        self.appendWayPoints = wayPoints
        self:startPathPlanningTo(targetNode, vecToNextPoint)
    end

    return self.resultPath
end

function MGPF:startPathPlanningTo(target, targetVector)
    --[[ Rough Idea:
        Create a box shape with length of ~0.5m and width and height of the vehicle (+ small buffer)
        Start by scanning in front of the vehicle.

        Now start snaking your way to the target similar to Dubins (as in step by step with small incremental turns instead of 45°/90° angles)

        At each point, you have 3 options: Left, Center, Right.
        Always choose the one that leads closest to the target and is free/unrestricted.

        If all three are deadends, backtrace in the created tree, until you have a fork that isn't yet traversed in each three directions and start over.

        Open question: How to apply the constrain of the target heading?
        1. Create a tree with x branches from the target backwards for y steps and then make sure to head for the closest one of these leave nodes.
        2. Just make a target point that is x meters behind and once reached, check only a Dubins style approach (can you turn towards the final location and heading, given the current pos and heading)
            Target is x, but y is approached and when inside y Box, start a Dubins search. If that fails, then what?
        
                 |-------|
                 |       |
        x        |   y   |
                 |       |        
                 |-------|
    --]]

    
    print("startPathPlanningTo " .. target.x .. " / " .. target.z)
    print("targetVector " .. targetVector.x .. " / " .. targetVector.z)

    self.tree = {}
    self.onFinalApproach = false
    self.vehicleNode = self.vehicle.components[1].node
    self.width, self.length = AutoDrive.getVehicleDimensions(self.vehicle, false)
    self.width = self.width * MGPF.WIDTH_ADDITON
    self.height = self.vehicle.size.height
    self.radius = AutoDrive.getDriverRadius(self.vehicle, true) * MGPF.RADIUS_ADDITION
    self.target = target
    self.targetVector = AutoDrive.normalizeVector(targetVector)
    print("self.targetVector " .. self.targetVector.x .. " / " .. self.targetVector.z)
    print("self.radius " .. self.radius)
    self.approachTarget = {
        x = self.target.x - self.radius * 2 * self.targetVector.x,
        z = self.target.z - self.radius * 2 * self.targetVector.z
    }
    
    self.theta = math.acos( 1 - ( math.pow(self.stepSize, 2) / (2 * math.pow(self.radius, 2)) ))
    self.stepX = - (self.radius - (math.cos(self.theta) * self.radius))
    self.stepZ = math.sin(self.theta) * self.radius


    local firstLeave = {}
    firstLeave.x, firstLeave.y, firstLeave.z = localToWorld(self.vehicleNode, 0, 0, self.length / 2 + self.stepSize)
    local rx, _, rz = localDirectionToWorld(self.vehicleNode, 0, 0, 1)
    firstLeave.ry = math.atan2(rx, rz)
    firstLeave.scaleIndex = 0
    firstLeave.restricted = self:checkLeave(firstLeave)
    firstLeave.parent = nil
    firstLeave.depth = 0
    

    ADDrawingManager:addLineTask(firstLeave.x, firstLeave.y, firstLeave.z, self.approachTarget.x, AutoDrive:getTerrainHeightAtWorldPos(self.approachTarget.x, self.approachTarget.z), self.approachTarget.z, 1, 0, 0, 1)

    local leave = firstLeave
    local stepCount = 0
    local reachedTargetPoint = false
    while stepCount < MGPF.MAX_STEPS and reachedTargetPoint == false and leave ~= nil do
        reachedTargetPoint = self:reachedTarget(leave)

        if reachedTargetPoint then
            print("startPathPlanningTo " .. target.x .. " / " .. target.z .. " reachedTargetPoint")
            break
        end
        
        self:createNextLeaves(leave)

        leave = self:getNextLeaveToFollow(leave)
        stepCount = stepCount + 1
    end

    if reachedTargetPoint then
        -- Generate a path now
        print("Generate a path now")
        self.chainTargetToStart = {}
        local index = 0
        while leave ~= nil do            
            index = index + 1
            print("chainTargetToStart " .. " index: " .. index .. " at depth: " .. leave.depth)
            self.chainTargetToStart[index] = leave
            leave = leave.parent
        end

        self.resultPath = {}
        for reversedIndex = 0, (index-1), 1 do
            print("resultPath " .. " reversedIndex: " .. reversedIndex .. " at depth: " .. self.chainTargetToStart[index - reversedIndex].depth)
            self.resultPath[reversedIndex + 1] = self.chainTargetToStart[index - reversedIndex]
        end

        for i, wp in self.appendWayPoints do         
            index = index + 1
            self.resultPath[index] = wp
        end
        
        self:drawResultingPath()
    else
        self.resultPath = nil
    end

    print("startPathPlanningTo " .. target.x .. " / " .. target.z .. " done")
end

function MGPF:checkLeave(leave)
    self.hit = false
    leave.restricted = false
    local usedWidth = self.width * (1 + leave.scaleIndex * MGPF.WIDTH_SCALER) 
    overlapBox(leave.x, leave.y + self.height / 2 + self.groundClearance, leave.z, 0, leave.ry, 0, usedWidth / 2, self.height / 2, self.stepSize / 2, "collisionTestCallback", self, self.mask, true, true, true, true)
      
    if self.hit then
        DebugUtil.drawOverlapBox(leave.x, leave.y + self.height / 2 + self.groundClearance, leave.z, 0, leave.ry, 0, usedWidth / 2, self.height / 2, self.stepSize / 2, 1, 0, 0)
        leave.restricted = true
    else
        DebugUtil.drawOverlapBox(leave.x, leave.y + self.height / 2 + self.groundClearance, leave.z, 0, leave.ry, 0, usedWidth / 2, self.height / 2, self.stepSize / 2, 1, 1, 1)
    end
end

function MGPF:collisionTestCallback(transformId)
    local collisionObject = g_currentMission.nodeToObject[transformId]

    if collisionObject == nil then
        -- let try if parent is an object
        local parent = getParent(transformId)
        if parent then
            collisionObject = g_currentMission.nodeToObject[parent]
        end
    end

    if collisionObject ~= nil then
        if collisionObject ~= self and collisionObject ~= self.vehicle and not AutoDrive:checkIsConnected(self.vehicle:getRootVehicle(), collisionObject) then            
            self.hit = true
            return true
        end
    else
        self.hit = true
        return true
    end

    return false
end 

function MGPF:createNextLeaves(leave)
    local rotationY = -leave.ry
    local worldStepXLeft = self.stepX * math.cos(rotationY) - self.stepZ * math.sin(rotationY)
    local worldStepZLeft = self.stepX * math.sin(rotationY) + self.stepZ * math.cos(rotationY)

    local worldStepXRight = -self.stepX * math.cos(rotationY) - self.stepZ * math.sin(rotationY)
    local worldStepZRight = -self.stepX * math.sin(rotationY) + self.stepZ * math.cos(rotationY)

    local worldStepXForward = - self.stepSize * math.sin(rotationY)
    local worldStepZForward =   self.stepSize * math.cos(rotationY)
    
    local leftPoint = {
        x = leave.x + worldStepXLeft,
        y = leave.y,
        z = leave.z + worldStepZLeft
    }

    local rightPoint = {
        x = leave.x + worldStepXRight,
        y = leave.y,
        z = leave.z + worldStepZRight
    }

    local centerPoint = {
        x = leave.x + worldStepXForward,
        y = leave.y,
        z = leave.z + worldStepZForward
    }

    leave.left = self:createLeaveAt(leftPoint.x, leftPoint.y, leftPoint.z, AutoDrive.normalizeAngle(leave.ry - self.theta), leave, 1)
    leave.right = self:createLeaveAt(rightPoint.x, rightPoint.y, rightPoint.z, AutoDrive.normalizeAngle(leave.ry + self.theta), leave, 1)
    leave.forward = self:createLeaveAt(centerPoint.x, centerPoint.y, centerPoint.z, leave.ry, leave, -1)
end

function MGPF:createLeaveAt(x, y, z, rotY, parent, scaleIndexDiff)
    local leave = {}
    leave.x = x
    leave.y = AutoDrive:getTerrainHeightAtWorldPos(x, z)
    leave.z = z
    leave.ry = rotY
    leave.scaleIndex = math.clamp(parent.scaleIndex + scaleIndexDiff, 0, 10)
    self:checkLeave(leave)
    leave.parent = parent
    leave.depth = parent.depth + 1

    return leave
end

function MGPF:reachedTarget(leave)
    -- check for target rotation here as well
    if not self.onFinalApproach then
        if self:distanceToTarget(leave) < (self.radius * 2) then
            self.onFinalApproach = true
        end
        return false
    else
        if leave.parent ~= nil then
            local vectorIncoming =  {x = leave.x - leave.parent.x, z = leave.z - leave.parent.z}
            local angleToTarget = math.abs(AutoDrive.angleBetween(vectorIncoming, self.targetVector))
            if self:distanceToTarget(leave) < 3 then
                print("AngleToTarget: " .. angleToTarget)
                if angleToTarget < 45 then
                    return true
                else
                    return false
                end
            end
        end
        return self:distanceToTarget(leave) < 3
    end    
end

function MGPF:distanceToTarget(leave)
    local target = self.approachTarget
    if self.onFinalApproach then
        target = self.target
    end
    local dx = leave.x - target.x
    local dz = leave.z - target.z
    
    return math.sqrt(dx * dx + dz * dz)
end

function MGPF:getNextLeaveToFollow(leave)
    local minDistance = math.huge
    local nextLeave = nil
    if leave.left.restricted == false then
        minDistance = self:distanceToTarget(leave.left)
        nextLeave = leave.left
        --print("Next leave at " .. leave.depth .. ": left with minDistance: " .. minDistance)
    end

    local rightDistance = self:distanceToTarget(leave.right)
    --print("RightDistance: " .. rightDistance .. " right.restricted: " .. tostring(leave.right.restricted))
    if leave.right.restricted == false and rightDistance < minDistance then
        minDistance = rightDistance
        nextLeave = leave.right
        --print("Next leave at " .. leave.depth .. ": right with minDistance: " .. minDistance)
    end

    local forwardDistance = self:distanceToTarget(leave.forward)
    --print("forwardDistance: " .. forwardDistance .. " forward.restricted: " .. tostring(leave.forward.restricted))
    if leave.forward.restricted == false and forwardDistance < minDistance then
        minDistance = forwardDistance
        nextLeave = leave.forward
        --print("Next leave at " .. leave.depth .. ": forward with minDistance: " .. minDistance)
    end
    
    if nextLeave ~= nil then
        return nextLeave
    end

    --print("Next leave at " .. leave.depth .. ": is nil. All restricted")

    if leave.parent == nil then
        --print("Next leave at " .. leave.depth .. ": is nil. Parent also nil. Abort here")
        return nil
    end

    -- All paths are blocked. Set current leave as restricted to mark this whole path as blocked
    -- Then backtrace recursively until the next open fork    
    leave.restricted = true
    
    --print("Next leave at " .. leave.depth .. ": is nil. Backtracing now")
    return self:getNextLeaveToFollow(leave.parent)    
end

function MGPF:drawResultingPath() 
    print("drawResultingPath 1")
    if self.resultPath == nil then
        return
    end 

    print("drawResultingPath 2")

    local lastPoint = nil
    for index, point in ipairs(self.resultPath) do
        print("drawResultingPath 3")
        if lastPoint ~= nil then
            print("drawResultingPath 4")
            ADDrawingManager:addLineTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
            ADDrawingManager:addArrowTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)

            if AutoDrive.getSettingState("lineHeight") == 1 then
                local gy = point.y - AutoDrive.drawHeight + 4
                local ty = lastPoint.y - AutoDrive.drawHeight + 4
                ADDrawingManager:addLineTask(point.x, gy, point.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
                ADDrawingManager:addSphereTask(point.x, gy, point.z, 3, 1, 0.09, 0.09, 0.15)
                ADDrawingManager:addLineTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, 1, 0.09, 0.09)
                ADDrawingManager:addArrowTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)
            else
                local gy = point.y - AutoDrive.drawHeight - 4
                local ty = lastPoint.y - AutoDrive.drawHeight - 4
                ADDrawingManager:addLineTask(point.x, gy, point.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
                ADDrawingManager:addSphereTask(point.x, gy, point.z, 3, 1, 0.09, 0.09, 0.15)
                ADDrawingManager:addLineTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, 1, 0.09, 0.09)
                ADDrawingManager:addArrowTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)
            end
        end
        lastPoint = point
    end
end