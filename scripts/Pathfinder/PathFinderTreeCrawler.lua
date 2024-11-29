PathFinderTreeCrawler = {}

PathFinderTreeCrawler.MAX_STEPS = 1000
PathFinderTreeCrawler.WIDTH_ADDITON = 1.35
PathFinderTreeCrawler.RADIUS_ADDITION = 1.35
PathFinderTreeCrawler.WIDTH_SCALER = 0.05


PathFinderTreeCrawler.MIN_FRUIT_VALUE = 10

--[[ Rough Idea:
        Create a box shape with length of ~0.5m and width and height of the vehicle (+ small buffer)
        Start by scanning in front of the vehicle.

        Now start snaking your way to the target similar to Dubins (as in step by step with small incremental turns instead of 45°/90° angles)

        At each point, you have 3 options: Left, Center, Right.
        Always choose the one that leads closest to the target and is free/unrestricted.

        If all three are deadends, backtrace in the created tree, until you have a fork that isn't yet traversed in each three directions and start over.

        How to apply the constrain of the target heading?
        For now: Just make a target point that is 2 x radius behind and once reached, let it run towards the actual target
        
                 |-------|
                 |       |
        x        |   y   |
                 |       |        
                 |-------|
--]]

function PathFinderTreeCrawler:new(vehicle)
    local o = {}
	setmetatable(o, self)
    self.__index = self

    o.mask = CollisionFlag.DEFAULT + CollisionFlag.STATIC_OBJECT + CollisionFlag.DYNAMIC_OBJECT + CollisionFlag.VEHICLE + CollisionFlag.TERRAIN_DELTA + CollisionFlag.TREE + CollisionFlag.BUILDING + CollisionFlag.WATER
    o.stepSize = 3.0    
    o.groundClearance = 0.5
    o.vehicle = vehicle
    
    return o
end

function PathFinderTreeCrawler:startPathPlanningTo(pathfinderTask)
    --targetPoint, targetVector, toNetwork, toPipe, fruitToCheckFor, wayPointsToAppend, fallBackMode, chasingVehicle, isSecondChasingVehicle    
    print("startPathPlanningTo " .. pathfinderTask.targetPoint.x .. " / " .. pathfinderTask.targetPoint.z)
    self.task = pathfinderTask
    self.tree = {}
    self.onFinalApproach = false
    self.vehicleNode = self.vehicle.components[1].node
    self.width, self.length = AutoDrive.getVehicleDimensions(self.vehicle, false)
    self.width = self.width * PathFinderTreeCrawler.WIDTH_ADDITON
    self.height = self.vehicle.size.height
    self.radius = AutoDrive.getDriverRadius(self.vehicle, true) * PathFinderTreeCrawler.RADIUS_ADDITION
    self.target = pathfinderTask.targetPoint
    self.targetVector = AutoDrive.normalizeVector(pathfinderTask.targetVector)
    self.approachTarget = {
        x = self.target.x - self.radius * 2 * self.targetVector.x,
        z = self.target.z - self.radius * 2 * self.targetVector.z
    }
    self.goingToNetwork = pathfinderTask.toNetwork
    self.fallBackMode = pathfinderTask.fallBackMode

    local vehicleWorldX, vehicleWorldY, vehicleWorldZ = localToWorld(self.vehicleNode, 0, 0, 0)
    local startIsOnField = AutoDrive.checkIsOnField(vehicleWorldX, vehicleWorldY, vehicleWorldZ) and self.vehicle.ad.sensors.frontSensorField:pollInfo(true)
    local endIsOnField = AutoDrive.checkIsOnField(self.target.x, 0, self.target.z)
    self.restrictToField = AutoDrive.getSetting("restrictToField", self.vehicle) and startIsOnField and endIsOnField and (not (self.fallBackMode > PathFinderModule.NO_FALLBACK))
    self.avoidFruitSetting = AutoDrive.getSetting("avoidFruit", self.vehicle) and (not (self.fallBackMode == PathFinderModule.FALLBACK_FRUIT))
    

    self.theta = math.acos( 1 - ( math.pow(self.stepSize, 2) / (2 * math.pow(self.radius, 2)) ))
    self.stepX = - (self.radius - (math.cos(self.theta) * self.radius))
    self.stepZ = math.sin(self.theta) * self.radius  

    local rx, _, rz = localDirectionToWorld(self.vehicleNode, 0, 0, 1)
    local ry = math.atan2(rx, rz)
    local startPointX, startPointY, startPointZ = localToWorld(self.vehicleNode, 0, 0, self.length / 2)-- + self.stepSize)
    local firstLeave = self:createLeaveAt(startPointX, startPointY, startPointZ, ry, nil, 0)
    
    ADDrawingManager:addLineTask(firstLeave.x, firstLeave.y, firstLeave.z, self.approachTarget.x, AutoDrive:getTerrainHeightAtWorldPos(self.approachTarget.x, self.approachTarget.z), self.approachTarget.z, 1, 0, 0, 1)

    local leave = firstLeave
    local stepCount = 0
    local reachedTargetPoint = false
    while stepCount < PathFinderTreeCrawler.MAX_STEPS and reachedTargetPoint == false and leave ~= nil do
        reachedTargetPoint = self:reachedTarget(leave)

        if reachedTargetPoint then
            print("startPathPlanningTo " .. self.target.x .. " / " .. self.target.z .. " reachedTargetPoint")
            break
        end
         
        if leave.left == nil then
            self:createNextLeaves(leave)
        end

        leave = self:getNextLeaveToFollow(leave)        
        --print("getNextLeaveToFollow - leave.depth: " .. leave.depth)
        stepCount = stepCount + 1
    end

    if reachedTargetPoint then
        -- Generate a path now
        self.chainTargetToStart = {}
        local index = 0
        while leave ~= nil do            
            index = index + 1
            self.chainTargetToStart[index] = leave
            leave = leave.parent
        end

        self.resultPath = {}
        for reversedIndex = 0, (index-1), 1 do
            self.resultPath[reversedIndex + 1] = self.chainTargetToStart[index - reversedIndex]
        end

        for i, wp in pathfinderTask.wayPointsToAppend do         
            index = index + 1
            self.resultPath[index] = wp
        end
        
        self:drawResultingPath()
    else
        self.resultPath = nil
    end

    print("startPathPlanningTo " .. self.target.x .. " / " .. self.target.z .. " done")
end

function PathFinderTreeCrawler:update()
   -- Nothing to do here
    if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        self:startPathPlanningTo(self.task)
        if self.resultPath ~= nil then
            self:drawResultingPath()
        end
    end    
end

function PathFinderTreeCrawler:getPath()
    return self.resultPath
end

function PathFinderTreeCrawler:hasFinished()
    if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        return false
    end
    return true
end

function PathFinderTreeCrawler:isBlocked()
    return self.resultPath == nil
end

function PathFinderTreeCrawler:createNextLeaves(leave)
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
    
    if leave.restricted then
        leave.left.restricted = true
        leave.right.restricted = true
        leave.forward.restricted = true    
    end
end

function PathFinderTreeCrawler:createLeaveAt(x, y, z, rotY, parent, scaleIndexDiff)
    local leave = {}
    leave.x = x
    leave.y = AutoDrive:getTerrainHeightAtWorldPos(x, z)
    leave.z = z
    leave.ry = rotY
    if parent == nil then
        leave.scaleIndex = 0
        leave.depth = 0        
    else
        leave.scaleIndex = math.clamp(parent.scaleIndex + scaleIndexDiff, 0, 10)
        leave.depth = parent.depth + 1        
    end

    leave.width = self.width * (1 + leave.scaleIndex * PathFinderTreeCrawler.WIDTH_SCALER)
    leave.vectorX = {x =   math.cos(leave.ry) * leave.width,        z = math.sin(leave.ry) * leave.width}
    leave.vectorZ = {x = - math.sin(leave.ry) * self.stepSize,      z = math.cos(leave.ry) * self.stepSize}
    leave.corners = {}
    leave.corners[1] =  {x = x + (-leave.vectorX.x/2 - leave.vectorZ.x/2), z = z + (-leave.vectorX.z/2 - leave.vectorZ.z/2)}
    leave.corners[2] =  {x = x + ( leave.vectorX.x/2 - leave.vectorZ.x/2), z = z + ( leave.vectorX.z/2 - leave.vectorZ.z/2)}
    leave.corners[3] =  {x = x + (-leave.vectorX.x/2 + leave.vectorZ.x/2), z = z + (-leave.vectorX.z/2 + leave.vectorZ.z/2)}
    leave.corners[4] =  {x = x + ( leave.vectorX.x/2 + leave.vectorZ.x/2), z = z + ( leave.vectorX.z/2 + leave.vectorZ.z/2)}
    leave.parent = parent
    self:checkLeave(leave)

    local gridKey = string.format("%.1f|%.1f", leave.x, leave.z)
    --print("creating leave with gridkey: " .. gridKey)
    if self.tree.gridKey ~= nil then
        --print("Found leave at same location")
        self.restriced = true
    else
        self.tree.gridKey = true        
    end

    return leave
end

function PathFinderTreeCrawler:reachedTarget(leave)
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
                --print("AngleToTarget: " .. angleToTarget)
                if angleToTarget < 45 then
                    return true
                else
                    leave.restriced = true
                    return false
                end
            end
        end
        return self:distanceToTarget(leave) < 3
    end    
end

function PathFinderTreeCrawler:distanceToTarget(leave)
    local target = self.approachTarget
    if self.onFinalApproach then
        target = self.target
    end
    local dx = leave.x - target.x
    local dz = leave.z - target.z
    
    return math.sqrt(dx * dx + dz * dz)
end

function PathFinderTreeCrawler:getNextLeaveToFollow(leave)
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

function PathFinderTreeCrawler:checkLeave(leave)
    --print("Start checking leave at depth: " .. leave.depth)
    leave.restricted = self.restrictToField and AutoDrive.checkIsOnField(leave.x, 0, leave.z)
    --print("Leave is restricted due to field: " .. tostring(leave.restricted) .. " self.restricted: " .. tostring(self.restrictToField))

    if not leave.restricted and self.avoidFruitSetting then
        self:checkForFruitAtLeave(leave)
        --print("Leave is restricted due to fruit: " .. tostring(leave.restricted) .. " fruitValue: " .. leave.fruitValue)    
    end

    if not leave.restricted and leave.parent ~= nil then
        -- check for up/down is to big or below water level
        --local length = MathUtil.vector3Length(leave.x - leave.parent.x, leave.y - leave.parent.y, leave.z - leave.parent.z)
        local slopeAngle = math.atan2(math.abs(leave.y - leave.parent.y) , self.stepSize)   
        leave.restricted = math.abs(slopeAngle) > PathFinderModule.SLOPE_DETECTION_THRESHOLD   
        if leave.depth < 5 then
            print("leave.depth: " .. leave.depth .. " slopeAngle: " .. slopeAngle .. " threshold: " .. PathFinderModule.SLOPE_DETECTION_THRESHOLD)            
        end
        --leave.restricted = self:checkSlopeAngle(leave.x, leave.z, leave.parent.x, leave.parent.z)
    end

    if not leave.restricted then
        self.hit = false
        overlapBox(leave.x, leave.y + self.height / 2 + self.groundClearance, leave.z, 0, leave.ry, 0, leave.width / 2, self.height / 2, self.stepSize / 2, "collisionTestCallback", self, self.mask, true, true, true, true)
        leave.restricted = self.hit
        --print("Leave is restricted due to collision: " .. tostring(leave.restricted))
    end

    if leave.restricted then
        DebugUtil.drawOverlapBox(leave.x, leave.y + self.height / 2 + self.groundClearance, leave.z, 0, leave.ry, 0, leave.width / 2, self.height / 2, self.stepSize / 2, 1, 0, 0)
    else
        DebugUtil.drawOverlapBox(leave.x, leave.y + self.height / 2 + self.groundClearance, leave.z, 0, leave.ry, 0, leave.width / 2, self.height / 2, self.stepSize / 2, 1, 1, 1)
    end
end

function PathFinderTreeCrawler:collisionTestCallback(transformId)
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

function PathFinderTreeCrawler:checkForFruitAtLeave(leave)
    if self.goingToNetwork then
        -- on the way to network, check all fruit types
        self.fruitToCheck = nil
    end

    if self.fruitToCheck == nil then
        for _, fruitType in pairs(g_fruitTypeManager:getFruitTypes()) do
            if not (fruitType == g_fruitTypeManager:getFruitTypeByName("MEADOW")) then
                local fruitTypeIndex = fruitType.index
                self:checkForFruitTypeInLeave(leave, fruitTypeIndex)
            end
            --stop if cell is already restricted and/or fruit type is now known
            if leave.restricted ~= false or self.fruitToCheck ~= nil then
                break
            end
        end
    else
        self:checkForFruitTypeInLeave(leave, self.fruitToCheck)
    end
end

function PathFinderTreeCrawler:checkForFruitTypeInLeave(leave, fruitTypeIndex)    
    local fruitValue = 0
    fruitValue, _, _, _ = FSDensityMapUtil.getFruitArea(fruitTypeIndex, leave.corners[1].x, leave.corners[1].z, leave.corners[2].x, leave.corners[2].z, leave.corners[3].x, leave.corners[3].z, true, true)
    
    if (self.fruitToCheck == nil or self.fruitToCheck < 1) and (fruitValue > PathFinderTreeCrawler.MIN_FRUIT_VALUE) then
        self.fruitToCheck = fruitTypeIndex
    end
    leave.restricted = leave.restricted or (fruitValue > PathFinderTreeCrawler.MIN_FRUIT_VALUE)

    leave.hasFruit = (fruitValue > PathFinderTreeCrawler.MIN_FRUIT_VALUE)
    leave.fruitValue = fruitValue
end

function PathFinderTreeCrawler:checkSlopeAngle(x1, z1, x2, z2)
    local vectorFromPrevious = {x = x1 - x2, z = z1 - z2}
    local worldPosMiddle = {x = x2 + vectorFromPrevious.x / 2, z = z2 + vectorFromPrevious.z / 2}

    local terrain1 = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x1, 0, z1)
    local terrain2 = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x2, 0, z2)
    local length = MathUtil.vector3Length(x1 - x2, terrain1 - terrain2, z1 - z2)
    local angleBetween = math.atan(math.abs(terrain1 - terrain2) / length)

    local angleLeft = 0
    local angleRight = 0

    if self.cos90 == nil then
        -- speed up the calculation
        self.cos90 = math.cos(math.rad(90))
        self.sin90 = math.sin(math.rad(90))
        self.cos270 = math.cos(math.rad(270))
        self.sin270 = math.sin(math.rad(270))
    end

    local rotX = vectorFromPrevious.x * self.cos90 - vectorFromPrevious.z * self.sin90
    local rotZ = vectorFromPrevious.x * self.sin90 + vectorFromPrevious.z * self.cos90
    local vectorLeft = {x = rotX, z = rotZ}

    rotX = vectorFromPrevious.x * self.cos270 - vectorFromPrevious.z * self.sin270
    rotZ = vectorFromPrevious.x * self.sin270 + vectorFromPrevious.z * self.cos270
    local vectorRight = {x = rotX, z = rotZ}

    local worldPosLeft = {x = x1 + vectorLeft.x / 2, z = z1 + vectorLeft.z / 2}
    local worldPosRight = {x = x1 + vectorRight.x / 2, z = z1 + vectorRight.z / 2}
    local terrainLeft = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPosLeft.x, 0, worldPosLeft.z)
    local terrainRight = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPosRight.x, 0, worldPosRight.z)
    local lengthLeft = MathUtil.vector3Length(worldPosLeft.x - x1, terrainLeft - terrain1, worldPosLeft.z - z1)
    local lengthRight = MathUtil.vector3Length(worldPosRight.x - x1, terrainRight - terrain1, worldPosRight.z - z1)
    angleLeft = math.atan(math.abs(terrainLeft - terrain1) / lengthLeft)
    angleRight = math.atan(math.abs(terrainRight - terrain1) / lengthRight)

    local waterY = g_currentMission.environmentAreaSystem:getWaterYAtWorldPosition(worldPosMiddle.x, terrain3, worldPosMiddle.z) or -200

    local belowGroundLevel = terrain1 < waterY - 0.5 or terrain2 < waterY - 0.5 or terrain3 < waterY - 0.5   

    if belowGroundLevel 
    or (angleBetween) > PathFinderModule.SLOPE_DETECTION_THRESHOLD 
    or (angleLeft > PathFinderModule.SLOPE_DETECTION_THRESHOLD 
    or angleRight > PathFinderModule.SLOPE_DETECTION_THRESHOLD)
    then
        return true
    end
    return false
end

function PathFinderTreeCrawler:drawResultingPath() 
    if self.resultPath == nil then
        return
    end 

    local lastPoint = nil
    for index, point in ipairs(self.resultPath) do
        if lastPoint ~= nil then
            ADDrawingManager:addLineTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
            ADDrawingManager:addArrowTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)


            --if point.corners ~= nil then
              --  ADDrawingManager:addLineTask(point.corners[1].x, point.y, point.corners[1].z, point.corners[2].x, point.y, point.corners[2].z, 1, 1, 0.09, 0.09)
                --ADDrawingManager:addLineTask(point.corners[1].x, point.y, point.corners[1].z, point.corners[3].x, point.y, point.corners[3].z, 1, 0, 1, 0.09)
            --end

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