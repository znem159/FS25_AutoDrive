PathFinderAStarDubins = {}

function PathFinderAStarDubins:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle    
    o.dubins = ADDubins:new()
    PathFinderAStarDubins.reset(o)
    return o
end

function PathFinderAStarDubins:reset(vehicle)
    self.mask = AutoDrive.collisionMaskTerrain
    self.grid = {}
    self.wayPoints = {}
    self.initNew = false
    self.path = {}
    self.smoothDone = true
    self.fruitAreas = {}

    self.PP_UP = 0
    self.PP_UP_RIGHT = 1
    self.PP_RIGHT = 2
    self.PP_DOWN_RIGHT = 3
    self.PP_DOWN = 4
    self.PP_DOWN_LEFT = 5
    self.PP_LEFT = 6
    self.PP_UP_LEFT = 7
    self.direction_to_text = {
        "PP_UP",
        "PP_UP_RIGHT",
        "PP_RIGHT",
        "PP_DOWN_RIGHT",
        "PP_DOWN",
        "PP_DOWN_LEFT",
        "PP_LEFT",
        "PP_UP_LEFT",
        "unknown"
    }
    self.minTurnRadius = AutoDrive.getDriverRadius(self.vehicle) * 2 / 3
    self.dubinsDone = false
    self.dubinsCount = 0
end

function PathFinderAStarDubins:hasFinished()
    if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        return false
    end
    if self.isFinished and self.smoothDone == true then
        return true
    end
    return false
end

function PathFinderAStarDubins:getPath()
    return self.wayPoints
end

function PathFinderAStarDubins:startPathPlanningTo(pathfinderTask)
    --targetPoint, targetVector, toNetwork, toPipe, fruitToCheckFor, wayPointsToAppend, fallBackMode, chasingVehicle, isSecondChasingVehicle
    self.fallBackMode = pathfinderTask.fallBackMode
    self.targetVector = pathfinderTask.targetVector
    local vehicleWorldX, vehicleWorldY, vehicleWorldZ = getWorldTranslation(self.vehicle.components[1].node)
    local vehicleRx, _, vehicleRz = localDirectionToWorld(self.vehicle.components[1].node, 0, 0, 1)
    local vehicleVector = {x = vehicleRx, z = vehicleRz}
    self.startX = vehicleWorldX + PathFinderModule.PATHFINDER_START_DISTANCE * vehicleRx
    self.startZ = vehicleWorldZ + PathFinderModule.PATHFINDER_START_DISTANCE * vehicleRz
    self.start = {x = self.startX, z = self.startZ}

    local angleRad = math.atan2(pathfinderTask.targetVector.z, pathfinderTask.targetVector.x)
    angleRad = AutoDrive.normalizeAngle(angleRad)

    self.vectorX = {x =   math.cos(angleRad) * self.minTurnRadius, z = math.sin(angleRad) * self.minTurnRadius}
    self.vectorZ = {x = - math.sin(angleRad) * self.minTurnRadius, z = math.cos(angleRad) * self.minTurnRadius}

    --Make the target a few meters ahead of the road to the start point
    local targetX = pathfinderTask.targetPoint.x - math.cos(angleRad) * PathFinderModule.PATHFINDER_TARGET_DISTANCE
    local targetZ = pathfinderTask.targetPoint.z - math.sin(angleRad) * PathFinderModule.PATHFINDER_TARGET_DISTANCE

    self.steps = 0
    
    self.isFinished = false
    self.smoothStep = 0
    self.smoothDone = false
    self.max_pathfinder_steps = PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL * AutoDrive.getSetting("pathFinderTime")
    self.appendWayPoints = pathfinderTask.wayPointsToAppend

    local targetCellZ = (((targetX - self.startX) / self.vectorX.x) * self.vectorX.z - targetZ + self.startZ) / (((self.vectorZ.x / self.vectorX.x) * self.vectorX.z) - self.vectorZ.z)
    local targetCellX = (targetZ - self.startZ - targetCellZ * self.vectorZ.z) / self.vectorX.z
    targetCellX = AutoDrive.round(targetCellX)
    targetCellZ = AutoDrive.round(targetCellZ)
    self.targetCell = {x = targetCellX, z = targetCellZ, direction = self.PP_UP}
    self.targetAhead = {x = targetX + self.vectorX.x, z = targetZ + self.vectorX.z}
    self.targetAheadCell = self:worldLocationToGridLocation(self.targetAhead.x, self.targetAhead.z)


    self.q0 = {
        vehicleWorldX
        , -vehicleWorldZ
        , AutoDrive.normalizeAngle(math.atan2(vehicleRx, vehicleRz) + math.pi + math.pi / 2)
    }

    self.q1 = {
        targetX
        , -targetZ
        , AutoDrive.normalizeAngle(math.atan2(pathfinderTask.targetVector.x, pathfinderTask.targetVector.z) + math.pi + math.pi / 2)
    }

    self.startCell = {x = 0, z = 0}
    self.startCell.direction = self:worldDirectionToGridDirection(vehicleVector)
    self.startCell.visited = false
    self.startCell.out = nil
    self.startCell.isRestricted = false
    self.startCell.hasCollision = false
    self.startCell.hasFruit = false
    self.startCell.steps = 0
    self.startCell.bordercells = 0

    local vehicleBehindX, _, vehicleBehindZ = localDirectionToWorld(self.vehicle.components[1].node, 0, 0, -self.minTurnRadius)
    local vehicleBehindVector = {x = vehicleBehindX, z = vehicleBehindZ}
    self.behindStartCell = self:worldLocationToGridLocation(vehicleWorldX + vehicleBehindX, vehicleWorldZ + vehicleBehindZ)
    self.behindStartCell.direction = self:worldDirectionToGridDirection(vehicleBehindVector, vehicleVector)
    self.behind = {x = vehicleWorldX + vehicleBehindX, z = vehicleWorldZ + vehicleBehindZ}

    self.target = {x = targetX, z = targetZ}

    local targetCellZ = (((targetX - self.startX) / self.vectorX.x) * self.vectorX.z - targetZ + self.startZ) / (((self.vectorZ.x / self.vectorX.x) * self.vectorX.z) - self.vectorZ.z)
    local targetCellX = (targetZ - self.startZ - targetCellZ * self.vectorZ.z) / self.vectorX.z
    targetCellX = AutoDrive.round(targetCellX)
    targetCellZ = AutoDrive.round(targetCellZ)
    self.targetCell = {x = targetCellX, z = targetCellZ, direction = self.PP_UP}

    self.diffOverallNetTime = 0
    
    self:setupNew(self.behindStartCell, self.startCell,self.targetCell)
end

function PathFinderAStarDubins:setupNew(behindStartCell, startCell, targetCell, userdata)
    PathFinderModule.debugMsg(self.vehicle, "PFM:setupNew behindStartCell %s,%s startCell %s,%s targetCell %s,%s"
        , tostring(behindStartCell.x)
        , tostring(behindStartCell.z)
        , tostring(startCell.x)
        , tostring(startCell.z)
        , tostring(targetCell.x)
        , tostring(targetCell.z)
    )
    self.cachedNodes = {}
    self.openset = {}
    self.closedset = {}
    self.came_from = {}
    self.g_score = {}
    self.h_score = {}
    self.f_score = {}

    self.nodeBehindStart = self:get_node(behindStartCell.x, behindStartCell.z)
    self.nodeBehindStart.isBehind = true
    self.nodeBehindStart.direction = behindStartCell.direction

    self.nodeStart = self:get_node(startCell.x, startCell.z)
    self.nodeStart.isStart = true
    self.nodeStart.direction = startCell.direction

    self.nodeGoal = self:get_node(targetCell.x, targetCell.z)
    self.nodeGoal.isGoal = true
    self.nodeGoal.direction = targetCell.direction

    self.g_score[self.nodeBehindStart] = math.huge
    self.h_score[self.nodeBehindStart] = math.huge
    self.f_score[self.nodeBehindStart] = math.huge

    self.g_score[self.nodeStart] = 0
    self.h_score[self.nodeStart] = self:estimate_cost(self.nodeStart, self.nodeGoal)
    self.f_score[self.nodeStart] = self.h_score[self.nodeStart]
    self.came_from[self.nodeStart] = self.nodeBehindStart

    self.openset[self.nodeStart] = true
    self.closedset[self.nodeBehindStart] = true
    self:isDriveableAstar(self.nodeStart)
    self:setBlockedGoal()
    self.initNew = true
end

function PathFinderAStarDubins:isBlocked()
    return self.completelyBlocked or self.steps > (self.max_pathfinder_steps)
end

function PathFinderAStarDubins:update(dt)
    if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then        
        self:drawDebugNewPF()
        if self.isFinished and self.smoothDone and self.wayPoints ~= nil then
            local lastPoint = nil
            for index, point in ipairs(self.wayPoints) do
                if point.isPathFinderPoint and lastPoint ~= nil then
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
    end

    if self.isFinished then
        if not self.smoothDone then
            self:createWayPointsNew()
        end
        if self.smoothDone then
            ADScheduler:removePathfinderVehicle(self.vehicle)
        end
        return
    end

    self.steps = self.steps + 1
    if (self.steps % 100) == 0 then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - self.steps %d #self.grid %d", self.steps, table.count(self.grid))
    end

    if not self.isFinished then
        if MathUtil.vector2Length(self.start.x - self.target.x, self.start.z - self.target.z) < PathFinderModule.PATHFINDER_MIN_DISTANCE_START_TARGET then
            -- try dubins first before full pathfinder if close to target
            if not self.dubinsDone then
                self.dubinsCount = self.dubinsCount + 1
                local dubinsPath = self:getDubinsPath()
                PathFinderModule.debugMsg(self.vehicle, "PFM:update getDubinsPath dubinsPath %s"
                    , tostring(dubinsPath)
                )
                if dubinsPath then
                    self.dubinsDone = true
                    self.wayPoints = dubinsPath
                    self:appendWayPointsNew()
                    self.isFinished = true
                    self.smoothDone = true
                    return  -- found path
                else
                    -- self.completelyBlocked = true
                      -- no valid path
                    -- PathFinderModule.debugMsg(self.vehicle, "PFM:update getDubinsPath self.completelyBlocked %s"
                    --     , tostring(self.completelyBlocked)
                    -- )
                end
                PathFinderModule.debugMsg(self.vehicle, "PFM:update getDubinsPath self.fallBackMode3 %s"
                    , tostring(self.fallBackMode == PathFinderModule.FALLBACK_FRUIT)
                )
                if self.fallBackMode == PathFinderModule.FALLBACK_FRUIT or self.dubinsCount > 4 then
                    self.dubinsDone = true
                end
            end
        end
        if not self.initNew then
            self:setupNew(self.behindStartCell, self.startCell,self.targetCell)
            local dx, dz = self.nodeStart.x - self.nodeGoal.x, self.nodeStart.z - self.nodeGoal.z
            local diff = math.sqrt(dx * dx + dz * dz)
            local toCloseToTarget = (diff < 3)
            dx, dz = self.nodeBehindStart.x - self.nodeGoal.x, self.nodeBehindStart.z - self.nodeGoal.z
            diff = math.sqrt(dx * dx + dz * dz)
            toCloseToTarget = toCloseToTarget or (diff < 3)
            if (self.nodeBehindStart == self.nodeGoal) or toCloseToTarget then
                self.completelyBlocked = true
                return  -- no valid path
            end
        end
        local diffNetTime = netGetTime()

        local current
        local add_neighbor_fn = function(neighbor, cost)
            if self:isDriveableAstar(neighbor) then
                if not self.closedset[neighbor] then
                    if not cost then cost = self:get_cost(current, neighbor) end
                    local tentative_g_score = self.g_score[current] + cost
                    local openset_idx = self.openset[neighbor]
                    if not openset_idx or tentative_g_score < self.g_score[neighbor] then
                        self.came_from[neighbor] = current
                        self.g_score[neighbor] = tentative_g_score
                        self.h_score[neighbor] = self.h_score[neighbor] or self:estimate_cost(neighbor, self.nodeGoal)
                        self.f_score[neighbor] = tentative_g_score + self.h_score[neighbor]
                        self.openset[neighbor] = true
                    end
                end
            end
        end
        local count = 0
        while next(self.openset) do
            count = count + 1
            if count > 10000 then
                diffNetTime = netGetTime() - diffNetTime
                self.diffOverallNetTime = self.diffOverallNetTime + diffNetTime

                AutoDrive.debugMsg(self.vehicle, "PFM:find ERROR exit counter count %d self.diffOverallNetTime %d"
                    , count
                    , self.diffOverallNetTime
                )

                self.completelyBlocked = true
                return  -- no valid path
            end
            current = self:pop_best_node(self.openset, self.f_score)
            if current == self.nodeGoal or self:reachedGoal(current, self.nodeGoal) then
                self.came_from[self.nodeGoal] = current
                self.path = self:unwind_path({}, self.came_from, self.nodeGoal)
                table.insert(self.path, self.nodeGoal)
                diffNetTime = netGetTime() - diffNetTime
                self.diffOverallNetTime = self.diffOverallNetTime + diffNetTime
                if current then
                    PathFinderModule.debugMsg(self.vehicle, "PFM:update find goal reached self.steps %d diffOverallNetTime %d self.nodeGoal xz %d,%d current xz %d,%d"
                        , self.steps
                        , self.diffOverallNetTime
                        , self.nodeGoal.x, self.nodeGoal.z
                        , current.x, current.z
                    )
                end

                self.isFinished = true
                return  -- found path
            end
            if current then self.closedset[current] = true end
            local from_node = self.came_from[current]
            self:get_neighbors(current, from_node, add_neighbor_fn)
            if count > (ADScheduler:getStepsPerFrame() * PathFinderModule.NEW_PF_STEP_FACTOR) then
                diffNetTime = netGetTime() - diffNetTime
                self.diffOverallNetTime = self.diffOverallNetTime + diffNetTime

                PathFinderModule.debugMsg(self.vehicle, "PFM:find steps in frame count %d diffNetTime %d self.diffOverallNetTime %d"
                    , count
                    , diffNetTime
                    , self.diffOverallNetTime
                )

                return -- shedule
            end
        end
        diffNetTime = netGetTime() - diffNetTime
        self.diffOverallNetTime = self.diffOverallNetTime + diffNetTime

        PathFinderModule.debugMsg(self.vehicle, "PFM:find exit end count %d self.diffOverallNetTime %d"
            , count
            , self.diffOverallNetTime
        )

        self.completelyBlocked = true
        return -- no valid path
    end
end

function PathFinderAStarDubins:gridLocationToWorldLocation(cell)
    local result = {x = 0, z = 0}

    result.x = self.target.x + (cell.x - self.targetCell.x) * self.vectorX.x + (cell.z - self.targetCell.z) * self.vectorZ.x
    result.z = self.target.z + (cell.x - self.targetCell.x) * self.vectorX.z + (cell.z - self.targetCell.z) * self.vectorZ.z

    return result
end

function PathFinderAStarDubins:worldDirectionToGridDirection(vector, baseVector)
    local baseVector = baseVector or self.vectorX
    local angle = AutoDrive.angleBetween(baseVector, vector)

    local direction = math.floor(angle / 45)
    local remainder = angle % 45
    if remainder >= 22.5 then
        direction = (direction + 1)
    elseif remainder <= -22.5 then
        direction = (direction - 1)
    end

    if direction < 0 then
        direction = 8 + direction
    end

    return direction
end

function PathFinderAStarDubins:worldLocationToGridLocation(worldX, worldZ)
    local result = {x = 0, z = 0}

    result.z = (((worldX - self.startX) / self.vectorX.x) * self.vectorX.z - worldZ + self.startZ) / (((self.vectorZ.x / self.vectorX.x) * self.vectorX.z) - self.vectorZ.z)
    result.x = (worldZ - self.startZ - result.z * self.vectorZ.z) / self.vectorX.z

    result.x = AutoDrive.round(result.x)
    result.z = AutoDrive.round(result.z)

    return result
end

function PathFinderAStarDubins:cellDistance(cell)
    return MathUtil.vector2Length(self.targetCell.x - cell.x, self.targetCell.z - cell.z)
end

function PathFinderAStarDubins:checkForFruitInArea(cell, corners)

    if self.goingToNetwork then
        -- on the way to network, check all fruit types
        self.fruitToCheck = nil
    end
    if self.fruitToCheck == nil then
        for _, fruitType in pairs(g_fruitTypeManager:getFruitTypes()) do
            if not (fruitType == g_fruitTypeManager:getFruitTypeByName("MEADOW")) then
                local fruitTypeIndex = fruitType.index
                self:checkForFruitTypeInArea(cell, fruitTypeIndex, corners)
            end
            --stop if cell is already restricted and/or fruit type is now known
            if cell.isRestricted ~= false or self.fruitToCheck ~= nil then
                break
            end
        end
    else
        self:checkForFruitTypeInArea(cell, self.fruitToCheck, corners)
    end
end

function PathFinderAStarDubins:checkForFruitTypeInArea(cell, fruitTypeIndex, corners)
    local fruitValue = 0
    fruitValue, _, _, _ = FSDensityMapUtil.getFruitArea(fruitTypeIndex, corners[1].x, corners[1].z, corners[2].x, corners[2].z, corners[3].x, corners[3].z, true, true)

    if (self.fruitToCheck == nil or self.fruitToCheck < 1) and (fruitValue > PathFinderModule.MIN_FRUIT_VALUE) then
        self.fruitToCheck = fruitTypeIndex
    end
    local wasRestricted = cell.isRestricted
    cell.isRestricted = cell.isRestricted or (fruitValue > PathFinderModule.MIN_FRUIT_VALUE)

    cell.hasFruit = (fruitValue > PathFinderModule.MIN_FRUIT_VALUE)
    cell.fruitValue = fruitValue

    --Allow fruit in the last few grid cells
    if (self:cellDistance(cell) <= 3 and self.goingToPipe) then
        cell.isRestricted = false or wasRestricted
    end
end

function PathFinderAStarDubins:getShapeDefByDirectionType(cell, getDefault)
    local shapeDefinition = {}
    shapeDefinition.angleRad = math.atan2(-self.targetVector.z, self.targetVector.x)
    shapeDefinition.angleRad = AutoDrive.normalizeAngle(shapeDefinition.angleRad)
    local worldPos = self:gridLocationToWorldLocation(cell)
    shapeDefinition.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPos.x, 1, worldPos.z)
    shapeDefinition.height = self.vehicle.size.height --2.65

    if cell.direction == self.PP_UP or cell.direction == self.PP_DOWN or cell.direction == self.PP_RIGHT or cell.direction == self.PP_LEFT or cell.direction == -1 or getDefault ~= nil then
        --default size:
        shapeDefinition.x = worldPos.x
        shapeDefinition.z = worldPos.z
        shapeDefinition.widthX = self.minTurnRadius / 2
        shapeDefinition.widthZ = self.minTurnRadius / 2
    elseif cell.direction == self.PP_UP_RIGHT then
        local offsetX = (-self.vectorX.x) / 2 + (-self.vectorZ.x) / 4
        local offsetZ = (-self.vectorX.z) / 2 + (-self.vectorZ.z) / 4
        shapeDefinition.x = worldPos.x + offsetX
        shapeDefinition.z = worldPos.z + offsetZ
        shapeDefinition.widthX = (self.minTurnRadius / 2) + math.abs(offsetX)
        shapeDefinition.widthZ = self.minTurnRadius / 2 + math.abs(offsetZ)
    elseif cell.direction == self.PP_UP_LEFT then
        local offsetX = (-self.vectorX.x) / 2 + (self.vectorZ.x) / 4
        local offsetZ = (-self.vectorX.z) / 2 + (self.vectorZ.z) / 4
        shapeDefinition.x = worldPos.x + offsetX
        shapeDefinition.z = worldPos.z + offsetZ
        shapeDefinition.widthX = self.minTurnRadius / 2 + math.abs(offsetX)
        shapeDefinition.widthZ = self.minTurnRadius / 2 + math.abs(offsetZ)
    elseif cell.direction == self.PP_DOWN_RIGHT then
        local offsetX = (self.vectorX.x) / 2 + (-self.vectorZ.x) / 4
        local offsetZ = (self.vectorX.z) / 2 + (-self.vectorZ.z) / 4
        shapeDefinition.x = worldPos.x + offsetX
        shapeDefinition.z = worldPos.z + offsetZ
        shapeDefinition.widthX = self.minTurnRadius / 2 + math.abs(offsetX)
        shapeDefinition.widthZ = self.minTurnRadius / 2 + math.abs(offsetZ)
    elseif cell.direction == self.PP_DOWN_LEFT then
        local offsetX = (self.vectorX.x) / 2 + (self.vectorZ.x) / 4
        local offsetZ = (self.vectorX.z) / 2 + (self.vectorZ.z) / 4
        shapeDefinition.x = worldPos.x + offsetX
        shapeDefinition.z = worldPos.z + offsetZ
        shapeDefinition.widthX = self.minTurnRadius / 2 + math.abs(offsetX)
        shapeDefinition.widthZ = self.minTurnRadius / 2 + math.abs(offsetZ)
    end

    local increaseCellFactor = 1.15
    if cell.isOnField ~= nil and cell.isOnField == true then
        increaseCellFactor = 1 --0.8
    end
    shapeDefinition.widthX = shapeDefinition.widthX * increaseCellFactor
    shapeDefinition.widthZ = shapeDefinition.widthZ * increaseCellFactor

    local corners = self:getCornersFromShapeDefinition(shapeDefinition)
    if corners ~= nil then
        for _, corner in pairs(corners) do
            shapeDefinition.y = math.max(shapeDefinition.y, getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, corner.x, 1, corner.z))
        end
    end

    return shapeDefinition
end

function PathFinderAStarDubins:drawDebugNewPF()
    -- AStar
    if self.cachedNodes and #self.cachedNodes > 0 then
        for z, row in pairs(self.cachedNodes) do
            for x, node in pairs(row) do
                -- cell outline
                local gridFactor = PathFinderModule.GRID_SIZE_FACTOR
                if self.isSecondChasingVehicle then
                    gridFactor = PathFinderModule.GRID_SIZE_FACTOR_SECOND_UNLOADER
                end
                local corners = self:getCorners(node, {x = self.vectorX.x * gridFactor, z = self.vectorX.z * gridFactor}, {x = self.vectorZ.x * gridFactor, z = self.vectorZ.z * gridFactor})
                local tempY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, corners[1].x, 1, corners[1].z)
                if node.isOnField then
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[2].x, tempY, corners[2].z, 1, 0, 1, 0) -- green
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[4].x, tempY, corners[4].z, 1, 0, 1, 0)
                    ADDrawingManager:addLineTask(corners[3].x, tempY, corners[3].z, corners[4].x, tempY, corners[4].z, 1, 0, 1, 0)
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 0)
                else
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[2].x, tempY, corners[2].z, 1, 1, 0, 0) -- red
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 0)
                    ADDrawingManager:addLineTask(corners[3].x, tempY, corners[3].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 0)
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[3].x, tempY, corners[3].z, 1, 1, 0, 0)
                end
                if node.isRestricted then
                    if node.hasFruit then
                        ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 1) -- cyan
                    else
                        ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 1, 0, 0) -- red
                    end
                else
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 0) -- green
                end
                if node.hasCollision then
                    if node.hasVehicleCollision then
                        ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 1) -- blue
                    else
                        ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[4].x, tempY, corners[4].z, 1, 1, 1, 0) -- yellow
                    end
                end

                -- cell text
                if node.text == nil then
                    node.text = string.format("%d,%d", x, z)
                end
                local point = self:gridLocationToWorldLocation({x = node.x, z = node.z})
                Utils.renderTextAtWorldPosition(point.x, tempY + 3, point.z, node.text, getCorrectTextSize(0.013), 0)

                -- behind point
                if node.isBehind then
                    local text = string.format("B %d,%d", x, z)
                    Utils.renderTextAtWorldPosition(point.x, tempY + 4, point.z, text, getCorrectTextSize(0.013), 0)
                    ADDrawingManager:addSphereTask(self.behind.x, tempY + 3, self.behind.z, 6, 0, 0, 1, 0) -- blue
                end

                -- start point
                if node.isStart then
                    local text = string.format("S %d,%d", x, z)
                    Utils.renderTextAtWorldPosition(point.x, tempY + 5, point.z, text, getCorrectTextSize(0.013), 0)
                    ADDrawingManager:addSphereTask(self.startX, tempY + 3, self.startZ, 6, 0, 1, 0, 0) -- green
                end

                -- goal point            
                if node.isGoal then
                    local text = string.format("T %d,%d", x, z)
                    Utils.renderTextAtWorldPosition(point.x, tempY + 6, point.z, text, getCorrectTextSize(0.013), 0)
                    ADDrawingManager:addSphereTask(self.target.x, tempY + 3, self.target.z, 6, 1, 0, 0, 0) -- red
                end
            end
        end
    end

    -- Dubins Path
    if self.dubinsPath and #self.dubinsPath > 0 then

        local lastPoint = nil
        for index, point in ipairs(self.dubinsPath) do
            if lastPoint ~= nil then
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
    if self.dubinsNodes then
        local i = 0
        for z, row in pairs(self.dubinsNodes) do
            for x, node in pairs(row) do
                local corners = node.corners
                i = i + 1
                local text = string.format("%d",i)
                -- Utils.renderTextAtWorldPosition(x, node.worldPos.y + 3, z, text, getCorrectTextSize(0.013), 0)
                local tempY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, node.corners[1].x, 1, node.corners[1].z)
                if node.isOnField then
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[2].x, tempY, corners[2].z, 1, 0, 1, 0) -- green
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[4].x, tempY, corners[4].z, 1, 0, 1, 0)
                    ADDrawingManager:addLineTask(corners[3].x, tempY, corners[3].z, corners[4].x, tempY, corners[4].z, 1, 0, 1, 0)
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 0)
                else
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[2].x, tempY, corners[2].z, 1, 1, 0, 0) -- red
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 0)
                    ADDrawingManager:addLineTask(corners[3].x, tempY, corners[3].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 0)
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[3].x, tempY, corners[3].z, 1, 1, 0, 0)
                end
                if node.isRestricted then
                    if node.hasFruit then
                        ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 1) -- cyan
                    else
                        ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 1, 0, 0) -- red
                    end
                else
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 0) -- green
                end
                if node.hasCollision then
                    if node.hasVehicleCollision then
                        ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 1) -- blue
                    else
                        ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[4].x, tempY, corners[4].z, 1, 1, 1, 0) -- yellow
                    end
                end
                if node.fruitValue and node.fruitValue > 0 then
                    local text = string.format("%d",node.fruitValue)
                    Utils.renderTextAtWorldPosition(x, node.worldPos.y + 3, z, text, getCorrectTextSize(0.013), 0)
                end
            end
        end
    end
end

function PathFinderAStarDubins:isDriveableAstar(cell)
    cell.isRestricted = false
    cell.incoming = cell.from_node
    cell.hasCollision = false

    local worldPos = self:gridLocationToWorldLocation(cell)
    --Try going through the checks in a way that fast checks happen before slower ones which might then be skipped

    cell.isOnField = AutoDrive.checkIsOnField(worldPos.x, 0, worldPos.z)

    -- check the most probable restrictions on field first to prevent unneccessary checks
    if not cell.isRestricted and self.restrictToField and not (self.fallBackMode == PathFinderModule.FALLBACK_OFF_FIELD or self.fallBackMode == PathFinderModule.FALLBACK_OFF_FIELD_BORDER) then
        cell.isRestricted = cell.isRestricted or (not cell.isOnField)
        if not cell.isOnField then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableAstar not cell.isOnField xz %d,%d "
                , cell.x, cell.z
            )
        end
    end

    local gridFactor = PathFinderModule.GRID_SIZE_FACTOR
    if self.isSecondChasingVehicle then
        gridFactor = PathFinderModule.GRID_SIZE_FACTOR_SECOND_UNLOADER
    end
    local corners = self:getCorners(cell, {x = self.vectorX.x * gridFactor, z = self.vectorX.z * gridFactor}, {x = self.vectorZ.x * gridFactor, z = self.vectorZ.z * gridFactor})

    if not cell.isRestricted and self.avoidFruitSetting and not self.fallBackMode == PathFinderModule.FALLBACK_FRUIT then
        -- check for fruit
        self:checkForFruitInArea(cell, corners) -- set cell.isRestricted if fruit found
        table.insert(self.fruitAreas, corners)
        if cell.isRestricted then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableAstar cell.isRestricted xz %d,%d fruit found %s"
                , cell.x, cell.z
                , self.fruitToCheck
            )
        end
    end

    if not cell.isRestricted and cell.incoming ~= nil then
        -- check for up/down is to big or below water level
        local worldPosPrevious = self:gridLocationToWorldLocation(cell.incoming)
        local angelToSlope, angle = self:checkSlopeAngle(worldPos.x, worldPos.z, worldPosPrevious.x, worldPosPrevious.z)    --> true if up/down or roll is to big or below water level
        cell.angle = angle
        cell.hasCollision = cell.hasCollision or angelToSlope
        cell.isRestricted = cell.isRestricted or cell.hasCollision
        if angelToSlope then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableAstar angelToSlope xz %d,%d"
                , cell.x, cell.z
            )
        end
    end

    if not cell.isRestricted then
        -- check for obstacles
        local shapeDefinition = self:getShapeDefByDirectionType(cell)   --> return shape for the cell according to direction, on ground level, 2.65m height
        self.collisionhits = 0
        local shapes = overlapBox(shapeDefinition.x, shapeDefinition.y + 3, shapeDefinition.z, 0, shapeDefinition.angleRad, 0, shapeDefinition.widthX, 2.65, shapeDefinition.widthZ, "collisionTestCallback", self, self.mask, true, true, true, true)
        cell.hasCollision = cell.hasCollision or (self.collisionhits > 0)
        cell.isRestricted = cell.isRestricted or cell.hasCollision
        if cell.hasCollision then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableAstar cell.hasCollision xz %d,%d collision"
                , cell.x, cell.z
            )
        end
    end

    if not cell.isRestricted and cell.incoming ~= nil then
        local worldPosPrevious = self:gridLocationToWorldLocation(cell.incoming)
        local vectorX = worldPosPrevious.x - worldPos.x
        local vectorZ = worldPosPrevious.z - worldPos.z
        local dirVec = { x=vectorX, z = vectorZ}

        local cellUsedByVehiclePath = AutoDrive.checkForVehiclePathInBox(corners, self.minTurnRadius, self.vehicle, dirVec)
        cell.isRestricted = cell.isRestricted or cellUsedByVehiclePath
        self.blockedByOtherVehicle = self.blockedByOtherVehicle or cellUsedByVehiclePath
        if cellUsedByVehiclePath then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableAstar cellUsedByVehiclePath xz %d,%d vehicle"
                , cell.x, cell.z
            )
        end
    end
    return not(cell.isRestricted)
end

 -- Cost of two adjacent nodes
-- current, neighbor
function PathFinderAStarDubins:get_cost(from_node, to_node)
    local dx, dz = from_node.x - to_node.x, from_node.z - to_node.z
    return math.sqrt(dx * dx + dz * dz) + (from_node.cost + to_node.cost) * 0.5
end

-- For heuristic. Estimate cost of current node to goal node
-- neighbor, goal
function PathFinderAStarDubins:estimate_cost(node, goal_node)
    return self:get_cost(node, goal_node) * 1.5 + (node.cost + goal_node.cost) * 0.5
end

-- current = self:pop_best_node(self.openset, self.f_score)
-- return: node / nil
-- self.openset, f_score
function PathFinderAStarDubins:pop_best_node(set, score)
    local best, node = math.huge, nil

    for k, v in pairs(set) do
        local s = score[k]

        if s < best then
            best  = s or math.huge
            node = k
        end
    end
    if not node then return end
    set[node] = nil
    return node
end

-- {}, self.came_from, self.nodeGoal
function PathFinderAStarDubins:unwind_path(flat_path, came_from, goal)
    if came_from[goal] and (came_from[goal] ~= self.nodeGoal) then
		table.insert(flat_path, 1, came_from[goal])
		return self:unwind_path(flat_path, came_from, came_from[goal])
	else
        return flat_path
	end
end

-- Node must be able to check if they are the same
-- so the example cannot directly return a different table for same coord
function PathFinderAStarDubins:get_node(x, z)
    local row = self.cachedNodes[z]
    if not row then row = {}; self.cachedNodes[z] = row end
    local node = row[x]
    if not node then node = { x = x, z = z, cost = 0 }; row[x] = node end
    return node
end

function PathFinderAStarDubins:getDirections(fromNode, node)
    if node == nil then
        AutoDrive.debugMsg(self.vehicle, "PFM:getDirections ERROR fromNode %s node %s"
            , tostring(fromNode)
            , tostring(node)
        )
        return
    end
    local directions = {}


    if (fromNode == nil and node.direction == self.PP_RIGHT) or (fromNode and fromNode.x == node.x and fromNode.z < node.z) then
        directions[1] = { -1, 1 }
        directions[1].direction = self.PP_DOWN_RIGHT
        directions[2] = { 0, 1 }
        directions[2].direction = self.PP_RIGHT
        directions[3] = { 1, 1 }
        directions[3].direction = self.PP_UP_RIGHT
    elseif (fromNode == nil and node.direction == self.PP_LEFT) or (fromNode and fromNode.x == node.x and fromNode.z > node.z) then
        directions[1] = { -1, -1 }
        directions[1].direction = self.PP_DOWN_LEFT
        directions[2] = { 0, -1 }
        directions[2].direction = self.PP_LEFT
        directions[3] = { 1, -1 }
        directions[3].direction = self.PP_UP_LEFT
    elseif (fromNode == nil and node.direction == self.PP_UP) or (fromNode and fromNode.x < node.x and fromNode.z == node.z) then
        directions[1] = { 1, -1 }
        directions[1].direction = self.PP_UP_LEFT
        directions[2] = { 1, 0 }
        directions[2].direction = self.PP_UP
        directions[3] = { 1, 1 }
        directions[3].direction = self.PP_UP_RIGHT
    elseif (fromNode == nil and node.direction == self.PP_DOWN) or (fromNode and fromNode.x > node.x and fromNode.z == node.z) then
        directions[1] = { -1, -1 }
        directions[1].direction = self.PP_DOWN_LEFT
        directions[2] = { -1, 0 }
        directions[2].direction = self.PP_DOWN
        directions[3] = { -1, 1 }
        directions[3].direction = self.PP_DOWN_RIGHT
    elseif (fromNode == nil and node.direction == self.PP_UP_RIGHT) or (fromNode and fromNode.x < node.x and fromNode.z < node.z) then
        directions[1] = { 1, 0 }
        directions[1].direction = self.PP_UP
        directions[2] = { 1, 1 }
        directions[2].direction = self.PP_UP_RIGHT
        directions[3] = { 0, 1 }
        directions[3].direction = self.PP_RIGHT
    elseif (fromNode == nil and node.direction == self.PP_DOWN_LEFT) or (fromNode and fromNode.x > node.x and fromNode.z > node.z) then
        directions[1] = { 0, -1 }
        directions[1].direction = self.PP_LEFT
        directions[2] = { -1, -1 }
        directions[2].direction = self.PP_DOWN_LEFT
        directions[3] = { -1, 0 }
        directions[3].direction = self.PP_DOWN
    elseif (fromNode == nil and node.direction == self.PP_UP_LEFT) or (fromNode and fromNode.x < node.x and fromNode.z > node.z) then
        directions[1] = { 0, -1 }
        directions[1].direction = self.PP_LEFT
        directions[2] = { 1, -1 }
        directions[2].direction = self.PP_UP_LEFT
        directions[3] = { 1, 0 }
        directions[3].direction = self.PP_UP
    elseif (fromNode == nil and node.direction == self.PP_DOWN_RIGHT) or (fromNode and fromNode.x > node.x and fromNode.z < node.z) then
        directions[1] = { -1, 0 }
        directions[1].direction = self.PP_DOWN
        directions[2] = { -1, 1 }
        directions[2].direction = self.PP_DOWN_RIGHT
        directions[3] = { 0, 1 }
        directions[3].direction = self.PP_RIGHT
    else
        if fromNode then
            AutoDrive.debugMsg(self.vehicle, "PFM:getDirections ERROR fromNode xz %d,%d"
                , fromNode.x
                , fromNode.z
            )
        end
        AutoDrive.debugMsg(self.vehicle, "PFM:getDirections ERROR fromNode %s node xz %d,%d node.direction %s"
            , tostring(fromNode)
            , node.x, node.z
            , tostring(node.direction)
        )
    end

    return directions
end

-- Return all neighbor nodes. Means a target that can be moved from the current node
-- current, from_node, add_neighbor_fn
function PathFinderAStarDubins:get_neighbors(node, fromNode, add_neighbor_fn)
    local x, z = node.x, node.z
    local directions = self:getDirections(fromNode, node)
    if directions then
        for i, offset in ipairs(directions) do
            local tnode = self:get_node(x + offset[1], z + offset[2])
            tnode.direction = offset.direction
            tnode.from_node = fromNode
            add_neighbor_fn(tnode)
        end
    end
end

local all_neighbors_offset = {
    { -1, -1 }, { 0, -1 }, { 1, -1 },
    { -1, 0 },             { 1, 0 },
    { -1, 1 },  { 0, 1 },  { 1, 1 }
}

function PathFinderAStarDubins:setBlockedGoal()
    local x, z = self.targetAheadCell.x, self.targetAheadCell.z
    for i, offset in ipairs(all_neighbors_offset) do
        if not (self.targetCell.x == (x + offset[1]) and self.targetCell.z == (z + offset[2])) then
            local tnode = self:get_node(x + offset[1], z + offset[2])
            tnode.text = string.format("G %d,%d",x + offset[1], z + offset[2])
            tnode.isBlockedGoal = true
            self.closedset[tnode] = true
        end
    end
end

function PathFinderAStarDubins:reachedGoal(current, goal)
    if math.abs(current.x - goal.x) < 2 and math.abs(current.z - goal.z) < 2 then
        return true
    else
        return false
    end
end


function PathFinderAStarDubins:createWayPointsNew()    
    if self.smoothStep == 0 then
        self.wayPoints = {}
        for index, cell in ipairs(self.path) do
            self.wayPoints[index] = self:gridLocationToWorldLocation(cell)
            self.wayPoints[index].y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, self.wayPoints[index].x, 1, self.wayPoints[index].z)
            self.wayPoints[index].direction = cell.direction
        end
        -- remove zig zag line
        self:smoothResultingPPPath()
    end
    -- shortcut the path if possible
    self:smoothResultingPPPath_Refined()

    if self.smoothStep == 2 then
        self:appendWayPointsNew()
    end
end

function PathFinderAStarDubins:appendWayPointsNew()
    -- When going to network, dont turn actual road network nodes into pathFinderPoints
    if self.goingToNetwork then
        for i = 1, #self.wayPoints, 1 do
            self.wayPoints[i].isPathFinderPoint = true
        end
    end

    if self.appendWayPoints ~= nil then
        for i = 1, #self.appendWayPoints, 1 do
            self.wayPoints[#self.wayPoints + 1] = self.appendWayPoints[i]
        end
        self.smoothStep = 3
    end

    -- See comment above
    if not self.goingToNetwork then
        for i = 1, #self.wayPoints, 1 do
            self.wayPoints[i].isPathFinderPoint = true
        end
    end
end

function PathFinderAStarDubins:isDriveableDubins(cell)
    cell.isRestricted = false
    cell.hasCollision = false
    PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins start xz %d,%d "
        , cell.x, cell.z
    )

    --Try going through the checks in a way that fast checks happen before slower ones which might then be skipped

    cell.isOnField = AutoDrive.checkIsOnField(cell.worldPos.x, 0, cell.worldPos.z)

    -- check the most probable restrictions on field first to prevent unneccessary checks
    if not cell.isRestricted and self.restrictToField and not (self.fallBackMode == PathFinderModule.FALLBACK_OFF_FIELD or self.fallBackMode == PathFinderModule.FALLBACK_OFF_FIELD_BORDER) then
        cell.isRestricted = cell.isRestricted or (not cell.isOnField)
        if not cell.isOnField then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins not cell.isOnField xz %d,%d "
                , cell.x, cell.z
            )
        end
    end

    local angleRad = AutoDrive.normalizeAngle(cell.t)
    local sizeMax = self.vehicle.size.width / 2
    local vectorX = {x =   math.cos(angleRad) * sizeMax, z = math.sin(angleRad) * sizeMax}
    local vectorZ = {x = - math.sin(angleRad) * sizeMax, z = math.cos(angleRad) * sizeMax}

    local corners = {}
    local centerLocation = cell.worldPos
    corners[1] = {x = centerLocation.x + (-vectorX.x - vectorZ.x), z = centerLocation.z + (-vectorX.z - vectorZ.z)}
    corners[2] = {x = centerLocation.x + (vectorX.x - vectorZ.x), z = centerLocation.z + (vectorX.z - vectorZ.z)}
    corners[3] = {x = centerLocation.x + (-vectorX.x + vectorZ.x), z = centerLocation.z + (-vectorX.z + vectorZ.z)}
    corners[4] = {x = centerLocation.x + (vectorX.x + vectorZ.x), z = centerLocation.z + (vectorX.z + vectorZ.z)}
    cell.corners = corners

    if not cell.isRestricted and self.avoidFruitSetting and not self.fallBackMode == PathFinderModule.FALLBACK_FRUIT then
        -- check for fruit
        self:checkForFruitInArea(cell, corners) -- set cell.isRestricted if fruit found
        if cell.isRestricted then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins cell.isRestricted xz %d,%d fruit found %s"
                , cell.x, cell.z
                , self.fruitToCheck
            )
        end
    end

    if not cell.isRestricted and cell.incoming ~= nil then
        -- check for up/down is to big or below water level
        local worldPosPrevious = cell.incoming.worldPos
        local angelToSlope, angle = self:checkSlopeAngle(cell.worldPos.x, cell.worldPos.z, worldPosPrevious.x, worldPosPrevious.z)    --> true if up/down or roll is to big or below water level
        cell.angle = angle
        cell.hasCollision = cell.hasCollision or angelToSlope
        cell.isRestricted = cell.isRestricted or cell.hasCollision
        if angelToSlope then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins angelToSlope xz %d,%d"
                , cell.x, cell.z
            )
        end
    end

    if not cell.isRestricted then
        -- check for obstacles
        self.collisionhits = 0
        local shapes = overlapBox(cell.worldPos.x, cell.worldPos.y + 3, cell.worldPos.z, 0, cell.t, 0, sizeMax, 2.65, sizeMax, "collisionTestCallback", self, self.mask, true, true, true, true)
        cell.hasCollision = cell.hasCollision or (self.collisionhits > 0)
        cell.isRestricted = cell.isRestricted or cell.hasCollision
        if cell.hasCollision then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins cell.hasCollision xz %d,%d collision"
                , cell.x, cell.z
            )
        end
    end

    if not cell.isRestricted and cell.incoming ~= nil then
        local worldPosPrevious = cell.incoming.worldPos
        local vectorX = worldPosPrevious.x - cell.worldPos.x
        local vectorZ = worldPosPrevious.z - cell.worldPos.z
        local dirVec = { x=vectorX, z = vectorZ}

        local cellUsedByVehiclePath = AutoDrive.checkForVehiclePathInBox(corners, self.minTurnRadius, self.vehicle, dirVec)
        cell.isRestricted = cell.isRestricted or cellUsedByVehiclePath
        self.blockedByOtherVehicle = self.blockedByOtherVehicle or cellUsedByVehiclePath
        if cellUsedByVehiclePath then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins cellUsedByVehiclePath xz %d,%d vehicle"
                , cell.x, cell.z
            )
        end
    end
    if cell.isRestricted then
        PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins end isRestricted xz %d,%d "
            , cell.x, cell.z
        )
    end
    return not(cell.isRestricted)
end

function PathFinderAStarDubins:getDubinsPath()
    PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath start")
    self.dubinsPath = nil
    local result = ADDubins.EDUBNOPATH
    self.dubinsNodes  = {}
    local diffNetTime = netGetTime()

    local function get_node(x, z)
        local row = self.dubinsNodes[z]
        if not row then row = {}; self.dubinsNodes[z] = row end
        local node = row[x]
        if not node then node = { x = x, z = z, cost = 0 }; row[x] = node end
        return node
    end

    local function checkPath()
        local result = self.dubins:dubins_path_sample_many(ADDubins.DubinsPath, 1, self.dubins.createWayPoints)
        PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath dubins_path_sample_many result %d"
            , result
        )
        if result == ADDubins.EDUBOK then
            PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath dubins_path_sample_many #self.dubins.outPath %d"
            , #self.dubins.outPath
            )
            if self.dubins.outPath and #self.dubins.outPath > 0 then
                local fromCell = nil
                for i, wayPoint in ipairs(self.dubins.outPath) do
                    local cell = get_node(wayPoint.x, wayPoint.z)
                    cell.worldPos = {x = wayPoint.x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, wayPoint.x, 1, wayPoint.z), z = wayPoint.z}
                    cell.t = wayPoint.t
                    cell.incomming = fromCell
                    fromCell = cell
                    if not self:isDriveableDubins(cell) then
                        result = ADDubins.EDUBNOPATH
                        break
                    end
                end
            else
                result = ADDubins.EDUBNOPATH
            end
        end
        return result
    end

    self.dubins.outPath = {}
    result = self.dubins:dubins_shortest_path(ADDubins.DubinsPath, self.q0, self.q1, self.minTurnRadius)
    PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath dubins_shortest_path result %d"
        , result
    )
    if result == ADDubins.EDUBOK then
        result = checkPath()
        if result == ADDubins.EDUBOK then
            PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath found shortest path #self.dubins.outPath %d result %d"
                , #self.dubins.outPath
                , result
            )
            self.dubinsPath = self.dubins.outPath
            return self.dubinsPath
        end
    end

    for i = 1, 6, 1 do
        self.dubins.outPath = {}
        result = self.dubins:dubins_path(ADDubins.DubinsPath, self.q0, self.q1, self.minTurnRadius, i)
        PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath dubins_path i %d result %d"
            , i
            , result
        )
        if result == ADDubins.EDUBOK then
            result = checkPath()
        end
        if result == ADDubins.EDUBOK then
            break
        end
    end
    PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath #self.dubins.outPath %d result %d"
        , #self.dubins.outPath
        , result
    )
    if result == ADDubins.EDUBOK then
        self.dubinsPath = self.dubins.outPath
    end

    diffNetTime = netGetTime() - diffNetTime
    PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath end diffNetTime %d"
        , diffNetTime
    )
    return self.dubinsPath
end


function PathFinderAStarDubins:getCornersFromShapeDefinition(shapeDefinition)
    local corners = {}
    corners[1] = {x = shapeDefinition.x + (-shapeDefinition.widthX), z = shapeDefinition.z + (-shapeDefinition.widthZ)}
    corners[2] = {x = shapeDefinition.x + (shapeDefinition.widthX), z = shapeDefinition.z + (shapeDefinition.widthZ)}
    corners[3] = {x = shapeDefinition.x + (-shapeDefinition.widthX), z = shapeDefinition.z + (shapeDefinition.widthZ)}
    corners[4] = {x = shapeDefinition.x + (shapeDefinition.widthX), z = shapeDefinition.z + (-shapeDefinition.widthZ)}

    return corners
end

function PathFinderAStarDubins:getCorners(cell, vectorX, vectorZ)
    local corners = {}
    local centerLocation = self:gridLocationToWorldLocation(cell)
    corners[1] = {x = centerLocation.x + (-vectorX.x - vectorZ.x), z = centerLocation.z + (-vectorX.z - vectorZ.z)}
    corners[2] = {x = centerLocation.x + (vectorX.x - vectorZ.x), z = centerLocation.z + (vectorX.z - vectorZ.z)}
    corners[3] = {x = centerLocation.x + (-vectorX.x + vectorZ.x), z = centerLocation.z + (-vectorX.z + vectorZ.z)}
    corners[4] = {x = centerLocation.x + (vectorX.x + vectorZ.x), z = centerLocation.z + (vectorX.z + vectorZ.z)}

    return corners
end

function PathFinderAStarDubins:createWayPoints()
    if self.smoothStep == 0 then
        local currentCell = self.targetCell
        self.chainTargetToStart = {}
        local index = 1
        self.chainTargetToStart[index] = currentCell
        index = index + 1
        while currentCell.x ~= 0 or currentCell.z ~= 0 do
            self.chainTargetToStart[index] = currentCell.incoming
            currentCell = currentCell.incoming
            if currentCell == nil then
                break
            end
            index = index + 1
        end
        index = index - 1

        self.chainStartToTarget = {}
        for reversedIndex = 0, index, 1 do
            self.chainStartToTarget[reversedIndex + 1] = self.chainTargetToStart[index - reversedIndex]
        end

        --Now build actual world coordinates as waypoints and include pre and append points
        self.wayPoints = {}
        for chainIndex, cell in pairs(self.chainStartToTarget) do
            self.wayPoints[chainIndex] = self:gridLocationToWorldLocation(cell)
            self.wayPoints[chainIndex].y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, self.wayPoints[chainIndex].x, 1, self.wayPoints[chainIndex].z)
            self.wayPoints[chainIndex].direction = cell.direction
        end

        -- remove zig zag line
        self:smoothResultingPPPath()
    end

    -- shortcut the path if possible
    self:smoothResultingPPPath_Refined()

    if self.smoothStep == 2 then
        -- When going to network, dont turn actual road network nodes into pathFinderPoints
        if self.goingToNetwork then
            for i = 1, #self.wayPoints, 1 do
                self.wayPoints[i].isPathFinderPoint = true
            end
        end

        if self.appendWayPoints ~= nil then
            for i = 1, #self.appendWayPoints, 1 do
                self.wayPoints[#self.wayPoints + 1] = self.appendWayPoints[i]
            end
            self.smoothStep = 3
            --PathFinderModule.debugVehicleMsg(self.vehicle,
                --string.format("PFM createWayPoints appendWayPoints %s",
                    --tostring(#self.appendWayPoints)
                --)
            --)
        end

        -- See comment above
        if not self.goingToNetwork then
            for i = 1, #self.wayPoints, 1 do
                self.wayPoints[i].isPathFinderPoint = true
            end
        end
    end
end

function PathFinderAStarDubins:smoothResultingPPPath()
    local index = 1
    local filteredIndex = 1
    local filteredWPs = {}

    while index < #self.wayPoints - 1 do
        local node = self.wayPoints[index]
        local nodeAhead = self.wayPoints[index + 1]
        local nodeTwoAhead = self.wayPoints[index + 2]

        filteredWPs[filteredIndex] = node
        filteredIndex = filteredIndex + 1

        if node.direction ~= nil and nodeAhead.direction ~= nil and nodeTwoAhead.direction ~= nil then
            if node.direction == nodeTwoAhead.direction and node.direction ~= nodeAhead.direction then
                index = index + 1 --skip next point because it is a zig zag line. Cut right through instead
            end
        end

        index = index + 1
    end

    while index <= #self.wayPoints do
        local node = self.wayPoints[index]
        filteredWPs[filteredIndex] = node
        filteredIndex = filteredIndex + 1
        index = index + 1
    end

    self.wayPoints = filteredWPs
    --PathFinderModule.debugVehicleMsg(self.vehicle,
        --string.format("PFM smoothResultingPPPath self.wayPoints %s",
            --tostring(#self.wayPoints)
        --)
    --)

end

function PathFinderAStarDubins:smoothResultingPPPath_Refined()
    if self.smoothStep == 0 then
        self.lookAheadIndex = 1
        self.smoothIndex = 1
        self.filteredIndex = 1
        self.filteredWPs = {}
        self.totalEagerSteps = 0

        --add first few without filtering
        while self.smoothIndex < #self.wayPoints and self.smoothIndex < 3 do
            self.filteredWPs[self.filteredIndex] = self.wayPoints[self.smoothIndex]
            self.filteredIndex = self.filteredIndex + 1
            self.smoothIndex = self.smoothIndex + 1
        end

        self.smoothStep = 1
    end

    local unfilteredEndPointCount = 5
    if self.smoothStep == 1 then
        local stepsThisFrame = 0
        while self.smoothIndex < #self.wayPoints - unfilteredEndPointCount and stepsThisFrame < ADScheduler:getStepsPerFrame() do

            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                PathFinderModule.debugVehicleMsg(self.vehicle,
                    string.format("PFM smoothResultingPPPath_Refined self.smoothIndex %d ",
                        self.smoothIndex
                    )
                )
            end
            stepsThisFrame = stepsThisFrame + 1

            local node = self.wayPoints[self.smoothIndex]
            local previousNode = nil
            local worldPos = self.wayPoints[self.smoothIndex]

            if self.totalEagerSteps == nil or self.totalEagerSteps == 0 then
                if self.filteredWPs[self.filteredIndex-1].x ~= node.x and self.filteredWPs[self.filteredIndex-1].z ~= node.z then
                    self.filteredWPs[self.filteredIndex] = node
                    if self.filteredIndex > 1 then
                        previousNode = self.filteredWPs[self.filteredIndex - 1]
                    end
                    self.filteredIndex = self.filteredIndex + 1

                    self.lookAheadIndex = 1
                    self.totalEagerSteps = 0
                end
            end

            local widthOfColBox = self.minTurnRadius
            local sideLength = widthOfColBox * PathFinderModule.GRID_SIZE_FACTOR
            local y = worldPos.y
            local foundCollision = false

            if stepsThisFrame > math.max(1, (ADScheduler:getStepsPerFrame() * 0.4)) then
                break
            end

            while (foundCollision == false) and ((self.smoothIndex + self.totalEagerSteps) < (#self.wayPoints - unfilteredEndPointCount)) and stepsThisFrame <= math.max(1, (ADScheduler:getStepsPerFrame() * 0.4)) do

                if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                    PathFinderModule.debugVehicleMsg(self.vehicle,
                        string.format("PFM smoothResultingPPPath_Refined self.smoothIndex %d self.totalEagerSteps %d",
                            self.smoothIndex,
                            self.totalEagerSteps
                        )
                    )
                end

                local hasCollision = false
                stepsThisFrame = stepsThisFrame + 1
                local nodeAhead = self.wayPoints[self.smoothIndex + self.totalEagerSteps + 1]
                local nodeTwoAhead = self.wayPoints[self.smoothIndex + self.totalEagerSteps + 2]
                if not hasCollision and nodeAhead and nodeTwoAhead then
                    local angle = AutoDrive.angleBetween({x = nodeAhead.x - node.x, z = nodeAhead.z - node.z}, {x = nodeTwoAhead.x - nodeAhead.x, z = nodeTwoAhead.z - nodeAhead.z})
                    angle = math.abs(angle)
                    if angle > 60 then
                        hasCollision = true

                        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                            PathFinderModule.debugVehicleMsg(self.vehicle,
                                string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                    1
                                )
                            )
                        end
                    end
                    if previousNode ~= nil then
                        angle = AutoDrive.angleBetween({x = node.x - previousNode.x, z = node.z - previousNode.z}, {x = nodeTwoAhead.x - node.x, z = nodeTwoAhead.z - node.z})
                        angle = math.abs(angle)
                        if angle > 60 then
                            hasCollision = true

                            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                                PathFinderModule.debugVehicleMsg(self.vehicle,
                                    string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                        2
                                    )
                                )
                            end
                        end
                        angle = AutoDrive.angleBetween({x = node.x - previousNode.x, z = node.z - previousNode.z}, {x = nodeAhead.x - node.x, z = nodeAhead.z - node.z})
                        angle = math.abs(angle)
                        if angle > 60 then
                            hasCollision = true

                            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                                PathFinderModule.debugVehicleMsg(self.vehicle,
                                    string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                        3
                                    )
                                )
                            end
                        end
                    end
                end

                if not hasCollision then
                    hasCollision = hasCollision or self:checkSlopeAngle(worldPos.x, worldPos.z, nodeAhead.x, nodeAhead.z)
                    if hasCollision then

                        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                            PathFinderModule.debugVehicleMsg(self.vehicle,
                                string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                    4
                                )
                            )
                        end
                    end
                end

                local vectorX = nodeAhead.x - node.x
                local vectorZ = nodeAhead.z - node.z
                local angleRad = math.atan2(-vectorZ, vectorX)
                angleRad = AutoDrive.normalizeAngle(angleRad)
                local length = math.sqrt(math.pow(vectorX, 2) + math.pow(vectorZ, 2)) + widthOfColBox

                local leftAngle = AutoDrive.normalizeAngle(angleRad + math.rad(-90))
                local rightAngle = AutoDrive.normalizeAngle(angleRad + math.rad(90))

                local cornerX = node.x - math.cos(leftAngle) * sideLength
                local cornerZ = node.z + math.sin(leftAngle) * sideLength

                local corner2X = nodeAhead.x - math.cos(leftAngle) * sideLength
                local corner2Z = nodeAhead.z + math.sin(leftAngle) * sideLength

                local corner3X = nodeAhead.x - math.cos(rightAngle) * sideLength
                local corner3Z = nodeAhead.z + math.sin(rightAngle) * sideLength

                local corner4X = node.x - math.cos(rightAngle) * sideLength
                local corner4Z = node.z + math.sin(rightAngle) * sideLength

                if not hasCollision then
                    self.collisionhits = 0
                    local shapes = overlapBox(worldPos.x + vectorX / 2, y + 3, worldPos.z + vectorZ / 2, 0, angleRad, 0, length / 2 + 2.5, 2.65, sideLength + 1.5, "collisionTestCallback", self, self.mask, true, true, true, true)
                    hasCollision = hasCollision or (self.collisionhits > 0)
                    
                    if hasCollision then
                        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                            PathFinderModule.debugVehicleMsg(self.vehicle,
                                string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                    5
                                )
                            )
                        end
                    end
                end

                if (self.smoothIndex > 1) then
                    local worldPosPrevious = self.wayPoints[self.smoothIndex - 1]
                    length = MathUtil.vector3Length(worldPos.x - worldPosPrevious.x, worldPos.y - worldPosPrevious.y, worldPos.z - worldPosPrevious.z)
                    local angleBetween = math.atan(math.abs(worldPos.y - worldPosPrevious.y) / length)

                    if (angleBetween) > PathFinderModule.SLOPE_DETECTION_THRESHOLD then
                        hasCollision = true

                        if hasCollision then
                            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                                PathFinderModule.debugVehicleMsg(self.vehicle,
                                    string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                        6
                                    )
                                )
                            end
                        end
                    end
                end

                if not hasCollision and self.avoidFruitSetting and not self.fallBackMode == PathFinderModule.FALLBACK_FRUIT then

                    local cornerWideX = node.x - math.cos(leftAngle) * sideLength * 4
                    local cornerWideZ = node.z + math.sin(leftAngle) * sideLength * 4

                    local cornerWide2X = nodeAhead.x - math.cos(leftAngle) * sideLength * 4
                    local cornerWide2Z = nodeAhead.z + math.sin(leftAngle) * sideLength * 4

                    local cornerWide4X = node.x - math.cos(rightAngle) * sideLength * 4
                    local cornerWide4Z = node.z + math.sin(rightAngle) * sideLength * 4

                    if self.goingToNetwork then
                        -- check for all fruit types
                        for _, fruitType in pairs(g_fruitTypeManager:getFruitTypes()) do
                            if not (fruitType == g_fruitTypeManager:getFruitTypeByName("MEADOW")) then
                                local fruitTypeIndex = fruitType.index
                                local fruitValue = 0
                                if self.isSecondChasingVehicle then
                                    fruitValue, _, _, _ = FSDensityMapUtil.getFruitArea(fruitTypeIndex, cornerWideX, cornerWideZ, cornerWide2X, cornerWide2Z, cornerWide4X, cornerWide4Z, true, true)
                                else
                                    fruitValue, _, _, _ = FSDensityMapUtil.getFruitArea(fruitTypeIndex, cornerX, cornerZ, corner2X, corner2Z, corner4X, corner4Z, true, true)
                                end
                                hasCollision = hasCollision or (fruitValue > 50)
                                if hasCollision then

                                    if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                                        PathFinderModule.debugVehicleMsg(self.vehicle,
                                            string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                                7
                                            )
                                        )
                                    end
                                    break
                                end
                            end
                        end
                    else
                        -- check only for fruit type detected on field
                        if self.fruitToCheck ~= nil then
                            local fruitValue = 0
                            if self.isSecondChasingVehicle then
                                fruitValue, _, _, _ = FSDensityMapUtil.getFruitArea(self.fruitToCheck, cornerWideX, cornerWideZ, cornerWide2X, cornerWide2Z, cornerWide4X, cornerWide4Z, true, true)
                            else
                                fruitValue, _, _, _ = FSDensityMapUtil.getFruitArea(self.fruitToCheck, cornerX, cornerZ, corner2X, corner2Z, corner4X, corner4Z, true, true)
                            end
                            hasCollision = hasCollision or (fruitValue > 50)

                            if hasCollision then
                                if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                                    PathFinderModule.debugVehicleMsg(self.vehicle,
                                        string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                            8
                                        )
                                    )
                                end
                            end
                        end
                    end
                end

                if not hasCollision then
                    local cellBox = AutoDrive.boundingBoxFromCorners(cornerX, cornerZ, corner2X, corner2Z, corner3X, corner3Z, corner4X, corner4Z)
                    hasCollision = hasCollision or AutoDrive.checkForVehiclePathInBox(cellBox, self.minTurnRadius, self.vehicle)

                    if hasCollision then
                        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                            PathFinderModule.debugVehicleMsg(self.vehicle,
                                string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                    9
                                )
                            )
                        end
                    end
                end

                foundCollision = hasCollision

                if foundCollision then
                    -- not used code removed
                else
                    self.lookAheadIndex = self.totalEagerSteps + 1
                end

                self.totalEagerSteps = self.totalEagerSteps + 1

                if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                    PathFinderModule.debugVehicleMsg(self.vehicle,
                        string.format("PFM smoothResultingPPPath_Refined self.smoothIndex %d self.totalEagerSteps %d self.filteredIndex %d foundCollision %s",
                            self.smoothIndex,
                            self.totalEagerSteps,
                            self.filteredIndex,
                            tostring(foundCollision)
                        )
                    )
                end
            end

            if foundCollision or ((self.smoothIndex + self.totalEagerSteps) >= (#self.wayPoints - unfilteredEndPointCount)) then
                self.smoothIndex = self.smoothIndex + math.max(1, (self.lookAheadIndex))
                self.totalEagerSteps = 0
            end
        end

        if self.smoothIndex >= #self.wayPoints - unfilteredEndPointCount then
            self.smoothStep = 2
        end
    end

    if self.smoothStep == 2 then
        --add remaining points without filtering
        while self.smoothIndex <= #self.wayPoints do
            local node = self.wayPoints[self.smoothIndex]
            self.filteredWPs[self.filteredIndex] = node
            self.filteredIndex = self.filteredIndex + 1
            self.smoothIndex = self.smoothIndex + 1
        end

        self.wayPoints = self.filteredWPs

        self.smoothDone = true

        PathFinderModule.debugVehicleMsg(self.vehicle,
            string.format("PFM smoothResultingPPPath_Refined self.wayPoints %s",
                tostring(#self.wayPoints)
            )
        )
    end
end

function PathFinderAStarDubins:checkSlopeAngle(x1, z1, x2, z2)
    local vectorFromPrevious = {x = x1 - x2, z = z1 - z2}
    local worldPosMiddle = {x = x2 + vectorFromPrevious.x / 2, z = z2 + vectorFromPrevious.z / 2}

    local terrain1 = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x1, 0, z1)
    local terrain2 = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x2, 0, z2)
    local terrain3 = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPosMiddle.x, 0, worldPosMiddle.z)
    local length = MathUtil.vector3Length(x1 - x2, terrain1 - terrain2, z1 - z2)
    local lengthMiddle = MathUtil.vector3Length(worldPosMiddle.x - x2, terrain3 - terrain2, worldPosMiddle.z - z2)
    local angleBetween = math.atan(math.abs(terrain1 - terrain2) / length)
    local angleBetweenCenter = math.atan(math.abs(terrain3 - terrain2) / lengthMiddle)

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

    local rotX = vectorFromPrevious.x * self.cos270 - vectorFromPrevious.z * self.sin270
    local rotZ = vectorFromPrevious.x * self.sin270 + vectorFromPrevious.z * self.cos270
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

    if belowGroundLevel then
        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM checkSlopeAngle belowGroundLevel x,z %d %d",
                    math.floor(x1),
                    math.floor(z1)
                )
            )
        end
        PathFinderModule.debugMsg(self.vehicle, "PFM:checkSlopeAngle belowGroundLevel xz %d,%d terrain123 %.1f %.1f %.1f getWaterYAtWorldPosition %s waterY %s "
        , math.floor(x1)
        , math.floor(z1)
        , terrain1
        , terrain2
        , terrain3
        , tostring(g_currentMission.environmentAreaSystem:getWaterYAtWorldPosition(worldPosMiddle.x, terrain3, worldPosMiddle.z))
        , tostring(waterY)
        )
    end

    if (angleBetween) > PathFinderModule.SLOPE_DETECTION_THRESHOLD then
        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM checkSlopeAngle (angleBetween * 1.25) > PathFinderModule.SLOPE_DETECTION_THRESHOLD  x,z %d %d",
                    math.floor(x1),
                    math.floor(z1)
                )
            )
        end
        PathFinderModule.debugMsg(self.vehicle, "PFM:checkSlopeAngle angleBetween xz %d,%d angleBetween %.1f terrain12 %.1f %.1f length %.1f "
        , math.floor(x1)
        , math.floor(z1)
        , math.deg(angleBetween)
        , terrain1
        , terrain2
        , length
        )
    end

    if (angleBetweenCenter) > PathFinderModule.SLOPE_DETECTION_THRESHOLD then
        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM checkSlopeAngle (angleBetweenCenter * 1.25) > PathFinderModule.SLOPE_DETECTION_THRESHOLD  x,z %d %d",
                    math.floor(x1),
                    math.floor(z1)
                )
            )
        end
        PathFinderModule.debugMsg(self.vehicle, "PFM:checkSlopeAngle angleBetweenCenter xz %d,%d angleBetweenCenter %.1f terrain32 %.1f %.1f lengthMiddle %.1f "
        , math.floor(x1)
        , math.floor(z1)
        , math.deg(angleBetweenCenter)
        , terrain3
        , terrain2
        , lengthMiddle
        )
    end

    if (angleLeft) > PathFinderModule.SLOPE_DETECTION_THRESHOLD then
        PathFinderModule.debugMsg(self.vehicle, "PFM:checkSlopeAngle angleLeft xz %d,%d angleLeft %.1f terrainLeft %.1f terrain1 %.1f lengthLeft %.1f "
        , math.floor(x1)
        , math.floor(z1)
        , math.deg(angleLeft)
        , terrainLeft
        , terrain1
        , lengthLeft
        )
    end

    if (angleRight) > PathFinderModule.SLOPE_DETECTION_THRESHOLD then
        PathFinderModule.debugMsg(self.vehicle, "PFM:checkSlopeAngle angleRight xz %d,%d angleRight %.1f terrainRight %.1f terrain1 %.1f lengthRight %.1f "
        , math.floor(x1)
        , math.floor(z1)
        , math.deg(angleRight)
        , terrainRight
        , terrain1
        , lengthRight
        )
    end

    if belowGroundLevel or (angleBetween) > PathFinderModule.SLOPE_DETECTION_THRESHOLD or (angleBetweenCenter) > PathFinderModule.SLOPE_DETECTION_THRESHOLD 
    or (angleLeft > PathFinderModule.SLOPE_DETECTION_THRESHOLD or angleRight > PathFinderModule.SLOPE_DETECTION_THRESHOLD)
    then
        return true, angleBetween
    end
    return false, angleBetween
end

function PathFinderAStarDubins:collisionTestCallback(transformId)
    if transformId ~= 0 and transformId ~= g_currentMission.terrainRootNode then
        local collisionObject = g_currentMission:getNodeObject(transformId)
        if (collisionObject == nil) or (collisionObject ~= nil and not (collisionObject.rootVehicle == self.vehicle)) then
            self.collisionhits = self.collisionhits + 1
            if PathFinderModule.debug == true then
                local currentCollMask = getCollisionMask(transformId)
                if currentCollMask then
                    local x, _, z = getWorldTranslation(transformId)
                    x = x + g_currentMission.mapWidth/2
                    z = z + g_currentMission.mapHeight/2

                    PathFinderModule.debugMsg(collisionObject, "PathFinderModule:collisionTestCallback transformId ->%s<- collisionObject ->%s<- getRigidBodyType->transformId %s getName->transformId %s getNodePath %s"
                        , tostring(transformId)
                        , tostring(collisionObject)
                        , tostring(getRigidBodyType(transformId))
                        , tostring(getName(transformId))
                        , tostring(I3DUtil.getNodePath(transformId))
                    )
                    if collisionObject then
                        PathFinderModule.debugMsg(collisionObject, "PathFinderModule:collisionTestCallback xmlFilename ->%s<-"
                            , tostring(collisionObject.xmlFilename)
                        )
                    end
                    PathFinderModule.debugMsg(collisionObject, "PathFinderModule:collisionTestCallback xz %.0f %.0f currentCollMask %s"
                        , x, z
                        , MathUtil.numberToSetBitsStr(currentCollMask)
                    )
                end
            end
        end
    end
end