PathFinderLegacy = {}

function PathFinderLegacy:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    PathFinderLegacy.reset(o)
    return o
end

function PathFinderLegacy:reset()
    self.mask = AutoDrive.collisionMaskTerrain
    self.steps = 0
    self.grid = {}
    self.wayPoints = {}
    self.initNew = false
    self.path = {}
    self.smoothDone = true
    self.fruitAreas = {}
    self.goingToNetwork  = false
    self.goingToPipe = false
    self.chasingVehicle = false
    self.isSecondChasingVehicle = false

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
end

function PathFinderLegacy:hasFinished()
    if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        return false
    end
    if self.isFinished and self.smoothDone == true then
        return true
    end
    return false
end

function PathFinderLegacy:startPathPlanningTo(pathfinderTask)
    --targetPoint, targetVector, toNetwork, toPipe, fruitToCheckFor, wayPointsToAppend, fallBackMode, chasingVehicle, isSecondChasingVehicle
    self.targetVector = pathfinderTask.targetVector
    local vehicleWorldX, vehicleWorldY, vehicleWorldZ = getWorldTranslation(self.vehicle.components[1].node)
    local vehicleRx, _, vehicleRz = localDirectionToWorld(self.vehicle.components[1].node, 0, 0, 1)
    local vehicleVector = {x = vehicleRx, z = vehicleRz}
    self.startX = vehicleWorldX + PathFinderModule.PATHFINDER_START_DISTANCE * vehicleRx
    self.startZ = vehicleWorldZ + PathFinderModule.PATHFINDER_START_DISTANCE * vehicleRz

    local angleRad = math.atan2(pathfinderTask.targetVector.z, pathfinderTask.targetVector.x)
    angleRad = AutoDrive.normalizeAngle(angleRad)

    self.vectorX = {x =   math.cos(angleRad) * self.minTurnRadius, z = math.sin(angleRad) * self.minTurnRadius}
    self.vectorZ = {x = - math.sin(angleRad) * self.minTurnRadius, z = math.cos(angleRad) * self.minTurnRadius}

    --Make the target a few meters ahead of the road to the start point
    local targetX = pathfinderTask.targetPoint.x - math.cos(angleRad) * PathFinderModule.PATHFINDER_TARGET_DISTANCE
    local targetZ = pathfinderTask.targetPoint.z - math.sin(angleRad) * PathFinderModule.PATHFINDER_TARGET_DISTANCE

    self.grid = {}
    self.steps = 0
    self.retryCounter = 0
    self.isFinished = false
    self.fallbackMode = pathfinderTask.fallBackMode
    self.max_pathfinder_steps = PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL * AutoDrive.getSetting("pathFinderTime")

    self.fruitToCheck = pathfinderTask.fruitToCheckFor
    self.goingToNetwork  = pathfinderTask.toNetwork
    self.goingToPipe = pathfinderTask.toPipe

    self.start = {x = self.startX, z = self.startZ}
    self.startCell = {x = 0, z = 0}
    self.startCell.direction = self:worldDirectionToGridDirection(vehicleVector)
    self.startCell.visited = false
    self.startCell.out = nil
    self.startCell.isRestricted = false
    self.startCell.hasCollision = false
    self.startCell.hasFruit = false
    self.startCell.steps = 0
    self.startCell.bordercells = 0
    self.currentCell = nil

    local vehicleBehindX, _, vehicleBehindZ = localDirectionToWorld(self.vehicle.components[1].node, 0, 0, -self.minTurnRadius)
    local vehicleBehindVector = {x = vehicleBehindX, z = vehicleBehindZ}
    self.behindStartCell = self:worldLocationToGridLocation(vehicleWorldX + vehicleBehindX, vehicleWorldZ + vehicleBehindZ)
    self.behindStartCell.direction = self:worldDirectionToGridDirection(vehicleBehindVector, vehicleVector)
    self.behind = {x = vehicleWorldX + vehicleBehindX, z = vehicleWorldZ + vehicleBehindZ}

    -- table.insert(self.grid, self.startCell)
    local gridKey = string.format("%d|%d|%d", self.startCell.x, self.startCell.z, self.startCell.direction)
    self.grid[gridKey] = self.startCell

    self.smoothStep = 0
    self.smoothDone = false
    self.target = {x = targetX, z = targetZ}

    local targetCellZ = (((targetX - self.startX) / self.vectorX.x) * self.vectorX.z - targetZ + self.startZ) / (((self.vectorZ.x / self.vectorX.x) * self.vectorX.z) - self.vectorZ.z)
    local targetCellX = (targetZ - self.startZ - targetCellZ * self.vectorZ.z) / self.vectorX.z
    targetCellX = AutoDrive.round(targetCellX)
    targetCellZ = AutoDrive.round(targetCellZ)
    self.targetCell = {x = targetCellX, z = targetCellZ, direction = self.PP_UP}
    self.targetAhead = {x = targetX + self.vectorX.x, z = targetZ + self.vectorX.z}
    self.targetAheadCell = self:worldLocationToGridLocation(self.targetAhead.x, self.targetAhead.z)

    self:determineBlockedCells(self.targetCell)

    self.appendWayPoints = pathfinderTask.wayPointsToAppend

    self.startIsOnField = AutoDrive.checkIsOnField(vehicleWorldX, vehicleWorldY, vehicleWorldZ) and self.vehicle.ad.sensors.frontSensorField:pollInfo(true)
    self.endIsOnField = AutoDrive.checkIsOnField(targetX, vehicleWorldY, targetZ)
    self.restrictToField = AutoDrive.getSetting("restrictToField", self.vehicle) and self.startIsOnField and self.endIsOnField

    self.chasingVehicle = pathfinderTask.chasingVehicle
    self.isSecondChasingVehicle = pathfinderTask.isSecondChasingVehicle
    self.completelyBlocked = false
    self.blockedByOtherVehicle = false
    self.avoidFruitSetting = AutoDrive.getSetting("avoidFruit", self.vehicle)

    if self.goingToPipe and math.sqrt(math.pow(vehicleWorldX - pathfinderTask.targetPoint.x, 2) + math.pow(vehicleWorldZ - pathfinderTask.targetPoint.z, 2)) < 50 then
        -- shorten path calculation for close combine
        self.max_pathfinder_steps = PathFinderModule.MAX_PATHFINDER_STEPS_COMBINE_TURN
    end

    self.chainStartToTarget = {}
end

function PathFinderLegacy:getPath()
    return self.wayPoints
end

function PathFinderLegacy:update(dt)
    if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then        
        if self.isFinished and self.smoothDone and self.wayPoints ~= nil and self.chainStartToTarget ~= nil and #self.chainStartToTarget > 0 and self.vehicle.ad.stateModule:getSpeedLimit() > 40 then
            self:drawDebugForCreatedRoute()
        else
            self:drawDebugForPF()
        end
    end

    if self.isFinished then
        if not self.smoothDone then
            self:createWayPoints()
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

    --We should see some perfomance increase by localizing the sqrt/pow functions right here
    local sqrt = math.sqrt
    local distanceFunc = function(a, b)
        return sqrt(a * a + b * b)
    end

    for i = 1, ADScheduler:getStepsPerFrame(), 1 do
        if self.currentCell == nil then

            self.currentCell = self:findClosestCell(self.grid, math.huge)

            if self.currentCell ~= nil and distanceFunc(self.targetCell.x - self.currentCell.x, self.targetCell.z - self.currentCell.z) < 1.5 then

                if self.currentCell.out == nil then
                    self:determineNextGridCells(self.currentCell)
                end

                if self:reachedTargetsNeighbor(self.currentCell.out) then
                    return
                end
            end

            if self.currentCell == nil then
                --Mark process stopped if we have no more cells to check
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - Mark process stopped if we have no more cells to check")
                PathFinderModule.debugVehicleMsg(self.vehicle,
                    string.format("PFM update - Mark process stopped if we have no more cells to check #self.grid %d",
                        table.count(self.grid)
                    )
                )
                self.completelyBlocked = true
                break
            end
        else
            if self.currentCell.out == nil then
                self:determineNextGridCells(self.currentCell)
            end
            self:testNextCells(self.currentCell)

            --Try shortcutting the process here. We dont have to go through the whole grid if one of the out points is viable and closer than the currenCell which was already closest
            local currentDistance = distanceFunc(self.targetCell.x - self.currentCell.x, self.targetCell.z - self.currentCell.z)

            local outCells = {}
            for _, outCell in pairs(self.currentCell.out) do
                local gridKey = string.format("%d|%d|%d", outCell.x, outCell.z, outCell.direction)
                if self.grid[gridKey] ~= nil then
                    table.insert(outCells, self.grid[gridKey])
                end
            end
            local nextCell = self:findClosestCell(outCells, currentDistance)

            -- Lets again check if we have reached our target already
            if self:reachedTargetsNeighbor(self.currentCell.out) then
                return
            end

            self.currentCell = nextCell
        end
    end
end

function PathFinderLegacy:isBlocked()
    return self.completelyBlocked or self.targetBlocked or self.steps > (self.max_pathfinder_steps)
end

function PathFinderLegacy:reachedTargetsNeighbor(cells)
    for _, outCell in pairs(cells) do
        if outCell.x == self.targetCell.x and outCell.z == self.targetCell.z then
            self.isFinished = true
            self.targetCell.incoming = self.currentCell --.incoming

            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - path found")
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM update - path found #self.grid %d",
                    table.count(self.grid)
                )
            )

            return true
        end
    end
    return false
end

function PathFinderLegacy:findClosestCell(cells, startDistance)
    local cellsToCheck = cells
    local sqrt = math.sqrt
    local distanceFunc = function(a, b)
        return sqrt(a * a + b * b)
    end
    local minDistance = startDistance
    local bestCell = nil
    local bestSteps = math.huge

    for _, cell in pairs(cellsToCheck) do
        if (not cell.visited) and (not cell.hasCollision) and (not cell.isRestricted) and (cell.bordercells < PathFinderModule.MAX_FIELDBORDER_CELLS) then
            local distance = distanceFunc(self.targetCell.x - cell.x, self.targetCell.z - cell.z)

            if (distance < minDistance) or (distance == minDistance and cell.steps < bestSteps) then
                minDistance = distance
                bestCell = cell
                bestSteps = cell.steps
            end
        end
    end

    return bestCell
end

function PathFinderLegacy:testNextCells(cell)
    for _, location in pairs(cell.out) do
        local createPoint = true
        local duplicatePointDirection = -1
        
        for i = -1, self.PP_UP_LEFT, 1 do -- important: do not break this loop to check for all directions!
            local gridKey = string.format("%d|%d|%d", location.x, location.z, i)
            if self.grid[gridKey] ~= nil then
                -- cell is already in the grid
                
                if self.grid[gridKey].x == location.x and self.grid[gridKey].z == location.z then     -- out cell is already in grid

                    if self.grid[gridKey].direction == -1 then
                        createPoint = false
                    elseif self.grid[gridKey].direction == location.direction then
                        createPoint = false
                        if self.grid[gridKey].steps > (cell.steps + 1) then --found shortcut
                            self.grid[gridKey].incoming = cell
                            self.grid[gridKey].steps = cell.steps + 1
                        end
                    end
                end
            end
        end
        if createPoint then
            local gridKey
            
            if duplicatePointDirection >= 0 then
                -- if different direction, it is not necessary to check the cell details again, just add a new entry in grid with known required restrictions
                -- Todo : Not true!! If we come from a different direction we ususally have a differently sized collision box to check. There is a difference between a 0° angle when coming from the last cell and a +/- 45° angle.
                gridKey = string.format("%d|%d|%d", location.x, location.z, duplicatePointDirection)
                location.isRestricted = self.grid[gridKey].isRestricted
                location.hasCollision = self.grid[gridKey].hasCollision
                location.bordercells = self.grid[gridKey].bordercells
                location.hasFruit = self.grid[gridKey].hasFruit
                location.fruitValue = self.grid[gridKey].fruitValue

                if not location.isRestricted and not location.hasCollision and location.incoming ~= nil then
                    -- check for up/down is to big or below water level
                    -- this is a required check as we come from different direction
                    local worldPos = self:gridLocationToWorldLocation(location)
                    local worldPosPrevious = self:gridLocationToWorldLocation(location.incoming)
                    location.hasCollision = location.hasCollision or self:checkSlopeAngle(worldPos.x, worldPos.z, worldPosPrevious.x, worldPosPrevious.z)    --> true if up/down is to big or below water level
                end
            else
                self:checkGridCell(location)
            end
            gridKey = string.format("%d|%d|%d", location.x, location.z, location.direction)
            self.grid[gridKey] = location
        end
    end

    cell.visited = true
end

function PathFinderLegacy:checkGridCell(cell)
    local worldPos = self:gridLocationToWorldLocation(cell)
    --Try going through the checks in a way that fast checks happen before slower ones which might then be skipped

    cell.isOnField = AutoDrive.checkIsOnField(worldPos.x, 0, worldPos.z)
    if self.restrictToField and (not self.fallBackMode == PathFinderModule.FALLBACK_OFF_FIELD_BORDER) then
        -- limit cells to field border only possible if started on field
        cell.bordercells = cell.incoming.bordercells + 1      -- by default we assume the new cell is not on field, so increase the counter

        if cell.incoming.bordercells == 0 then
            -- if incoming cell is on field we check if the new is also on field
            if cell.isOnField then
                -- still on field, so set the current cell counter to 0
                cell.bordercells = 0
            end
        end
    end

    -- check the most probable restrictions on field first to prevent unneccessary checks
    if not cell.isRestricted and self.restrictToField and not (self.fallBackMode == PathFinderModule.FALLBACK_OFF_FIELD or self.fallBackMode == PathFinderModule.FALLBACK_OFF_FIELD_BORDER) then
        -- in fallBackMode1 we ignore the field restriction
        cell.isRestricted = cell.isRestricted or (not cell.isOnField)
    end

    local gridFactor = PathFinderModule.GRID_SIZE_FACTOR * 1.3  --> 0.6
    if self.isSecondChasingVehicle then
        gridFactor = PathFinderModule.GRID_SIZE_FACTOR_SECOND_UNLOADER * 1.6    --> 1.7
    end
    local corners = self:getCorners(cell, {x = self.vectorX.x * gridFactor, z = self.vectorX.z * gridFactor}, {x = self.vectorZ.x * gridFactor, z = self.vectorZ.z * gridFactor})

    if not cell.isRestricted and self.avoidFruitSetting and not self.fallBackMode == PathFinderModule.FALLBACK_FRUIT then
        -- check for fruit
        self:checkForFruitInArea(cell, corners) -- set cell.isRestricted if fruit found
    end

    if not cell.isRestricted and cell.incoming ~= nil then
        -- check for up/down is to big or below water level
        local worldPosPrevious = self:gridLocationToWorldLocation(cell.incoming)
        local angelToSlope, angle = self:checkSlopeAngle(worldPos.x, worldPos.z, worldPosPrevious.x, worldPosPrevious.z)    --> true if up/down or roll is to big or below water level
        cell.angle = angle
        cell.hasCollision = cell.hasCollision or angelToSlope
    end

    if not cell.isRestricted and not cell.hasCollision then
        -- check for obstacles
        local shapeDefinition = self:getShapeDefByDirectionType(cell)   --> return shape for the cell according to direction, on ground level, 2.65m height
        local ignoreObstaclesUpToHeight = 0.5
        local shapes = overlapBox(shapeDefinition.x, shapeDefinition.y + ignoreObstaclesUpToHeight, shapeDefinition.z, 0, shapeDefinition.angleRad, 0, shapeDefinition.widthX, shapeDefinition.height - ignoreObstaclesUpToHeight, shapeDefinition.widthZ, "collisionTestCallback", self, self.mask, true, true, true, true)
        cell.hasCollision = cell.hasCollision or (shapes > 0)
    end

    if not cell.isRestricted and not cell.hasCollision and cell.incoming ~= nil then
        local worldPosPrevious = self:gridLocationToWorldLocation(cell.incoming)
        local vectorX = worldPosPrevious.x - worldPos.x
        local vectorZ = worldPosPrevious.z - worldPos.z
        local dirVec = { x=vectorX, z = vectorZ}

        local cellUsedByVehiclePath = AutoDrive.checkForVehiclePathInBox(corners, self.minTurnRadius, self.vehicle, dirVec)
        cell.isRestricted = cell.isRestricted or cellUsedByVehiclePath
        self.blockedByOtherVehicle = self.blockedByOtherVehicle or cellUsedByVehiclePath
    end
end

function PathFinderLegacy:collisionTestCallback(transformId)
    return true
end

function PathFinderLegacy:gridLocationToWorldLocation(cell)
    local result = {x = 0, z = 0}

    result.x = self.target.x + (cell.x - self.targetCell.x) * self.vectorX.x + (cell.z - self.targetCell.z) * self.vectorZ.x
    result.z = self.target.z + (cell.x - self.targetCell.x) * self.vectorX.z + (cell.z - self.targetCell.z) * self.vectorZ.z

    return result
end

function PathFinderLegacy:worldDirectionToGridDirection(vector, baseVector)
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

function PathFinderLegacy:worldLocationToGridLocation(worldX, worldZ)
    local result = {x = 0, z = 0}

    result.z = (((worldX - self.startX) / self.vectorX.x) * self.vectorX.z - worldZ + self.startZ) / (((self.vectorZ.x / self.vectorX.x) * self.vectorX.z) - self.vectorZ.z)
    result.x = (worldZ - self.startZ - result.z * self.vectorZ.z) / self.vectorX.z

    result.x = AutoDrive.round(result.x)
    result.z = AutoDrive.round(result.z)

    return result
end

function PathFinderLegacy:determineBlockedCells(cell)
    if (math.abs(cell.x) < 2 and math.abs(cell.z) < 2) then
        return
    end
    
    local gridKey = ""
    local direction = -1
    local x = 0
    local z = 0
    x = cell.x + 1
    z = cell.z + 0
    gridKey = string.format("%d|%d|%d", x, z, direction)
    self.grid[gridKey] = {x = x, z = z, direction = direction, isRestricted = true, hasCollision = true, steps = 100000, bordercells = 0}
    x = cell.x + 1
    z = cell.z - 1
    gridKey = string.format("%d|%d|%d", x, z, direction)
    self.grid[gridKey] = {x = x, z = z, direction = direction, isRestricted = true, hasCollision = true, steps = 100000, bordercells = 0}
    x = cell.x + 0
    z = cell.z + 1
    gridKey = string.format("%d|%d|%d", x, z, direction)
    self.grid[gridKey] = {x = x, z = z, direction = direction, isRestricted = true, hasCollision = true, steps = 100000, bordercells = 0}
    x = cell.x + 1
    z = cell.z + 1
    gridKey = string.format("%d|%d|%d", x, z, direction)
    self.grid[gridKey] = {x = x, z = z, direction = direction, isRestricted = true, hasCollision = true, steps = 100000, bordercells = 0}
    x = cell.x + 0
    z = cell.z - 1
    gridKey = string.format("%d|%d|%d", x, z, direction)
    self.grid[gridKey] = {x = x, z = z, direction = direction, isRestricted = true, hasCollision = true, steps = 100000, bordercells = 0}
end

function PathFinderLegacy:determineNextGridCells(cell)
    if cell.out == nil then
        cell.out = {}
    end
    if cell.direction == self.PP_UP then
        cell.out[1] = {x = cell.x + 1, z = cell.z - 1}
        cell.out[1].direction = self.PP_UP_LEFT
        cell.out[2] = {x = cell.x + 1, z = cell.z + 0}
        cell.out[2].direction = self.PP_UP
        cell.out[3] = {x = cell.x + 1, z = cell.z + 1}
        cell.out[3].direction = self.PP_UP_RIGHT
    elseif cell.direction == self.PP_UP_RIGHT then
        cell.out[1] = {x = cell.x + 1, z = cell.z + 0}
        cell.out[1].direction = self.PP_UP
        cell.out[2] = {x = cell.x + 1, z = cell.z + 1}
        cell.out[2].direction = self.PP_UP_RIGHT
        cell.out[3] = {x = cell.x + 0, z = cell.z + 1}
        cell.out[3].direction = self.PP_RIGHT
    elseif cell.direction == self.PP_RIGHT then
        cell.out[1] = {x = cell.x + 1, z = cell.z + 1}
        cell.out[1].direction = self.PP_UP_RIGHT
        cell.out[2] = {x = cell.x + 0, z = cell.z + 1}
        cell.out[2].direction = self.PP_RIGHT
        cell.out[3] = {x = cell.x - 1, z = cell.z + 1}
        cell.out[3].direction = self.PP_DOWN_RIGHT
    elseif cell.direction == self.PP_DOWN_RIGHT then
        cell.out[1] = {x = cell.x + 0, z = cell.z + 1}
        cell.out[1].direction = self.PP_RIGHT
        cell.out[2] = {x = cell.x - 1, z = cell.z + 1}
        cell.out[2].direction = self.PP_DOWN_RIGHT
        cell.out[3] = {x = cell.x - 1, z = cell.z + 0}
        cell.out[3].direction = self.PP_DOWN
    elseif cell.direction == self.PP_DOWN then
        cell.out[1] = {x = cell.x - 1, z = cell.z + 1}
        cell.out[1].direction = self.PP_DOWN_RIGHT
        cell.out[2] = {x = cell.x - 1, z = cell.z + 0}
        cell.out[2].direction = self.PP_DOWN
        cell.out[3] = {x = cell.x - 1, z = cell.z - 1}
        cell.out[3].direction = self.PP_DOWN_LEFT
    elseif cell.direction == self.PP_DOWN_LEFT then
        cell.out[1] = {x = cell.x - 1, z = cell.z - 0}
        cell.out[1].direction = self.PP_DOWN
        cell.out[2] = {x = cell.x - 1, z = cell.z - 1}
        cell.out[2].direction = self.PP_DOWN_LEFT
        cell.out[3] = {x = cell.x - 0, z = cell.z - 1}
        cell.out[3].direction = self.PP_LEFT
    elseif cell.direction == self.PP_LEFT then
        cell.out[1] = {x = cell.x - 1, z = cell.z - 1}
        cell.out[1].direction = self.PP_DOWN_LEFT
        cell.out[2] = {x = cell.x - 0, z = cell.z - 1}
        cell.out[2].direction = self.PP_LEFT
        cell.out[3] = {x = cell.x + 1, z = cell.z - 1}
        cell.out[3].direction = self.PP_UP_LEFT
    elseif cell.direction == self.PP_UP_LEFT then
        cell.out[1] = {x = cell.x - 0, z = cell.z - 1}
        cell.out[1].direction = self.PP_LEFT
        cell.out[2] = {x = cell.x + 1, z = cell.z - 1}
        cell.out[2].direction = self.PP_UP_LEFT
        cell.out[3] = {x = cell.x + 1, z = cell.z + 0}
        cell.out[3].direction = self.PP_UP
    end

    for _, outGoing in pairs(cell.out) do
        outGoing.visited = false
        outGoing.isRestricted = false
        outGoing.hasCollision = false
        outGoing.hasFruit = false
        outGoing.incoming = cell
        outGoing.steps = cell.steps + 1
        outGoing.bordercells = cell.bordercells
    end
end


function PathFinderLegacy:cellDistance(cell)
    return MathUtil.vector2Length(self.targetCell.x - cell.x, self.targetCell.z - cell.z)
end

function PathFinderLegacy:checkForFruitInArea(cell, corners)

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

function PathFinderLegacy:checkForFruitTypeInArea(cell, fruitTypeIndex, corners)
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

function PathFinderLegacy:drawDebugForPF()
    local AutoDriveDM = ADDrawingManager
    local pointTarget = self:gridLocationToWorldLocation(self.targetCell)
    local pointTargetUp = self:gridLocationToWorldLocation(self.targetCell)
    pointTarget.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointTarget.x, 1, pointTarget.z) + 3
    pointTargetUp.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointTargetUp.x, 1, pointTargetUp.z) + 20
    AutoDriveDM:addLineTask(pointTarget.x, pointTarget.y, pointTarget.z, pointTargetUp.x, pointTargetUp.y, pointTargetUp.z, 1, 0, 0, 1)
    local pointStart = {x = self.startX, z = self.startZ}
    pointStart.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointStart.x, 1, pointStart.z) + 3
    AutoDriveDM:addLineTask(pointStart.x, pointStart.y, pointStart.z, pointStart.x, pointStart.y + 20, pointStart.z, 1, 0, 1, 0)

    local color_red = 0.1
    local color_green = 0.1
    local color_blue = 0.1
    local color_count = 0
    local index = 0
    for _, cell in pairs(self.grid) do
        index = index + 1

        color_red = math.min(color_red + 0.25, 1)
        if color_red > 0.9 then
            color_green = math.min(color_green + 0.25, 1)
        end
        if color_green > 0.9 then
            color_blue = math.min(color_blue + 0.25, 1)
        end
        color_count = color_count + 1

        local worldPos = self:gridLocationToWorldLocation(cell)

        local shapeDefinition = self:getShapeDefByDirectionType(cell, true)   --> return shape for the cell according to direction, on ground level, 2.65m height
        local corners = self:getCornersFromShapeDefinition(shapeDefinition)
        local baseY = shapeDefinition.y + 3

        -- cell outline
        if cell.isOnField then
            ADDrawingManager:addLineTask(corners[1].x, baseY, corners[1].z, corners[3].x, baseY, corners[3].z, 1, 0, 1, 0) -- green
            ADDrawingManager:addLineTask(corners[3].x, baseY, corners[3].z, corners[2].x, baseY, corners[2].z, 1, 0, 1, 0)
            ADDrawingManager:addLineTask(corners[2].x, baseY, corners[2].z, corners[4].x, baseY, corners[4].z, 1, 0, 1, 0)
            ADDrawingManager:addLineTask(corners[4].x, baseY, corners[4].z, corners[1].x, baseY, corners[1].z, 1, 0, 1, 0)
        else
            ADDrawingManager:addLineTask(corners[1].x, baseY, corners[1].z, corners[3].x, baseY, corners[3].z, 1, 1, 0, 0) -- red
            ADDrawingManager:addLineTask(corners[3].x, baseY, corners[3].z, corners[2].x, baseY, corners[2].z, 1, 1, 0, 0)
            ADDrawingManager:addLineTask(corners[2].x, baseY, corners[2].z, corners[4].x, baseY, corners[4].z, 1, 1, 0, 0)
            ADDrawingManager:addLineTask(corners[4].x, baseY, corners[4].z, corners[1].x, baseY, corners[1].z, 1, 1, 0, 0)
        end
        
        if cell.isRestricted then
            if cell.hasFruit then
                ADDrawingManager:addLineTask(corners[1].x, baseY, corners[1].z, corners[2].x, baseY, corners[2].z, 1, 0, 1, 1) -- cyan
            else
                ADDrawingManager:addLineTask(corners[1].x, baseY, corners[1].z, corners[2].x, baseY, corners[2].z, 1, 1, 0, 0) -- red
            end
        else
            ADDrawingManager:addLineTask(corners[1].x, baseY, corners[1].z, corners[2].x, baseY, corners[2].z, 1, 0, 1, 0) -- green
        end
        if cell.hasCollision then
            if cell.hasVehicleCollision then
                ADDrawingManager:addLineTask(corners[3].x, baseY, corners[3].z, corners[4].x, baseY, corners[4].z, 1, 1, 0, 1) -- blue
            else
                ADDrawingManager:addLineTask(corners[3].x, baseY, corners[3].z, corners[4].x, baseY, corners[4].z, 1, 1, 1, 0) -- yellow
            end
        end
         
    end

    -- target cell marker
    local size = 0.3
    local pointA = self:gridLocationToWorldLocation(self.targetCell)
    pointA.x = pointA.x + self.vectorX.x * size + self.vectorZ.x * size
    pointA.z = pointA.z + self.vectorX.z * size + self.vectorZ.z * size
    pointA.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointA.x, 1, pointA.z) + 3
    local pointB = self:gridLocationToWorldLocation(self.targetCell)
    pointB.x = pointB.x - self.vectorX.x * size - self.vectorZ.x * size
    pointB.z = pointB.z - self.vectorX.z * size - self.vectorZ.z * size
    pointB.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointB.x, 1, pointB.z) + 3
    local pointC = self:gridLocationToWorldLocation(self.targetCell)
    pointC.x = pointC.x + self.vectorX.x * size - self.vectorZ.x * size
    pointC.z = pointC.z + self.vectorX.z * size - self.vectorZ.z * size
    pointC.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointC.x, 1, pointC.z) + 3
    local pointD = self:gridLocationToWorldLocation(self.targetCell)
    pointD.x = pointD.x - self.vectorX.x * size + self.vectorZ.x * size
    pointD.z = pointD.z - self.vectorX.z * size + self.vectorZ.z * size
    pointD.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointD.x, 1, pointD.z) + 3

    AutoDriveDM:addLineTask(pointA.x, pointA.y, pointA.z, pointB.x, pointB.y, pointB.z, 1, 1, 1, 1) -- white
    AutoDriveDM:addLineTask(pointC.x, pointC.y, pointC.z, pointD.x, pointD.y, pointD.z, 1, 1, 1, 1) -- white

    local pointAB = self:gridLocationToWorldLocation(self.targetCell)
    pointAB.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointAB.x, 1, pointAB.z) + 3

    local pointTargetVector = self:gridLocationToWorldLocation(self.targetCell)
    pointTargetVector.x = pointTargetVector.x + self.targetVector.x * 10
    pointTargetVector.z = pointTargetVector.z + self.targetVector.z * 10
    pointTargetVector.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointTargetVector.x, 1, pointTargetVector.z) + 3
    AutoDriveDM:addLineTask(pointAB.x, pointAB.y, pointAB.z, pointTargetVector.x, pointTargetVector.y, pointTargetVector.z, 1, 1, 1, 1) -- white
end

function PathFinderLegacy:drawDebugForCreatedRoute()
    local AutoDriveDM = ADDrawingManager
    if self.chainStartToTarget ~= nil then
        for _, cell in pairs(self.chainStartToTarget) do
            local shape = self:getShapeDefByDirectionType(cell)
            if shape.x ~= nil then
                local pointA = {
                    x = shape.x + shape.widthX * math.cos(shape.angleRad) + shape.widthZ * math.sin(shape.angleRad),
                    y = shape.y,
                    z = shape.z + shape.widthZ * math.cos(shape.angleRad) + shape.widthX * math.sin(shape.angleRad)
                }
                local pointB = {
                    x = shape.x - shape.widthX * math.cos(shape.angleRad) - shape.widthZ * math.sin(shape.angleRad),
                    y = shape.y,
                    z = shape.z + shape.widthZ * math.cos(shape.angleRad) + shape.widthX * math.sin(shape.angleRad)
                }
                local pointC = {
                    x = shape.x - shape.widthX * math.cos(shape.angleRad) - shape.widthZ * math.sin(shape.angleRad),
                    y = shape.y,
                    z = shape.z - shape.widthZ * math.cos(shape.angleRad) - shape.widthX * math.sin(shape.angleRad)
                }
                local pointD = {
                    x = shape.x + shape.widthX * math.cos(shape.angleRad) + shape.widthZ * math.sin(shape.angleRad),
                    y = shape.y,
                    z = shape.z - shape.widthZ * math.cos(shape.angleRad) - shape.widthX * math.sin(shape.angleRad)
                }

                AutoDriveDM:addLineTask(pointA.x, pointA.y, pointA.z, pointC.x, pointC.y, pointC.z, 1, 1, 1, 1)
                AutoDriveDM:addLineTask(pointB.x, pointB.y, pointB.z, pointD.x, pointD.y, pointD.z, 1, 1, 1, 1)

                if cell.incoming ~= nil then
                    local worldPos_cell = self:gridLocationToWorldLocation(cell)
                    local worldPos_incoming = self:gridLocationToWorldLocation(cell.incoming)

                    local vectorX = worldPos_cell.x - worldPos_incoming.x
                    local vectorZ = worldPos_cell.z - worldPos_incoming.z
                    local angleRad = math.atan2(-vectorZ, vectorX)
                    angleRad = AutoDrive.normalizeAngle(angleRad)
                    local widthOfColBox = math.sqrt(math.pow(self.minTurnRadius, 2) + math.pow(self.minTurnRadius, 2))
                    local sideLength = widthOfColBox / 2

                    local leftAngle = AutoDrive.normalizeAngle(angleRad + math.rad(-90))
                    local rightAngle = AutoDrive.normalizeAngle(angleRad + math.rad(90))

                    local cornerX = worldPos_incoming.x - math.cos(leftAngle) * sideLength
                    local cornerZ = worldPos_incoming.z + math.sin(leftAngle) * sideLength

                    local corner2X = worldPos_cell.x - math.cos(leftAngle) * sideLength
                    local corner2Z = worldPos_cell.z + math.sin(leftAngle) * sideLength

                    local corner3X = worldPos_cell.x - math.cos(rightAngle) * sideLength
                    local corner3Z = worldPos_cell.z + math.sin(rightAngle) * sideLength

                    local corner4X = worldPos_incoming.x - math.cos(rightAngle) * sideLength
                    local corner4Z = worldPos_incoming.z + math.sin(rightAngle) * sideLength

                    local inY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPos_incoming.x, 1, worldPos_incoming.z) + 1
                    local currentY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPos_cell.x, 1, worldPos_cell.z) + 1

                    AutoDriveDM:addLineTask(cornerX, inY, cornerZ, corner2X, currentY, corner2Z, 1, 1, 0, 0)
                    AutoDriveDM:addLineTask(corner2X, currentY, corner2Z, corner3X, currentY, corner3Z, 1, 1, 0, 0)
                    AutoDriveDM:addLineTask(corner3X, currentY, corner3Z, corner4X, inY, corner4Z, 1, 1, 0, 0)
                    AutoDriveDM:addLineTask(corner4X, inY, corner4Z, cornerX, inY, cornerZ, 1, 1, 0, 0)
                end
            end
        end
    end

    if self.wayPoints then
        for i, waypoint in pairs(self.wayPoints) do
            Utils.renderTextAtWorldPosition(waypoint.x, waypoint.y + 4, waypoint.z, "Node " .. i, getCorrectTextSize(0.013), 0)
            if i > 1 then
                local wp = waypoint
                local pfWp = self.wayPoints[i - 1]
                AutoDriveDM:addLineTask(wp.x, wp.y, wp.z, pfWp.x, pfWp.y, pfWp.z, 1, 0, 1, 1)
            end
        end
    end
end

function PathFinderLegacy:getShapeDefByDirectionType(cell, getDefault)
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

function PathFinderLegacy:getCornersFromShapeDefinition(shapeDefinition)
    local corners = {}
    corners[1] = {x = shapeDefinition.x + (-shapeDefinition.widthX), z = shapeDefinition.z + (-shapeDefinition.widthZ)}
    corners[2] = {x = shapeDefinition.x + (shapeDefinition.widthX), z = shapeDefinition.z + (shapeDefinition.widthZ)}
    corners[3] = {x = shapeDefinition.x + (-shapeDefinition.widthX), z = shapeDefinition.z + (shapeDefinition.widthZ)}
    corners[4] = {x = shapeDefinition.x + (shapeDefinition.widthX), z = shapeDefinition.z + (-shapeDefinition.widthZ)}

    return corners
end

function PathFinderLegacy:getCorners(cell, vectorX, vectorZ)
    local corners = {}
    local centerLocation = self:gridLocationToWorldLocation(cell)
    corners[1] = {x = centerLocation.x + (-vectorX.x - vectorZ.x), z = centerLocation.z + (-vectorX.z - vectorZ.z)}
    corners[2] = {x = centerLocation.x + (vectorX.x - vectorZ.x), z = centerLocation.z + (vectorX.z - vectorZ.z)}
    corners[3] = {x = centerLocation.x + (-vectorX.x + vectorZ.x), z = centerLocation.z + (-vectorX.z + vectorZ.z)}
    corners[4] = {x = centerLocation.x + (vectorX.x + vectorZ.x), z = centerLocation.z + (vectorX.z + vectorZ.z)}

    return corners
end

function PathFinderLegacy:createWayPoints()
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

function PathFinderLegacy:smoothResultingPPPath()
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

function PathFinderLegacy:smoothResultingPPPath_Refined()
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
                    local shapes = overlapBox(worldPos.x + vectorX / 2, y + 3, worldPos.z + vectorZ / 2, 0, angleRad, 0, length / 2 + 2.5, 2.65, sideLength + 1.5, "Ignore", nil, self.mask, true, true, true, true)
                    hasCollision = hasCollision or (shapes > 0)
                    

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

function PathFinderLegacy:checkSlopeAngle(x1, z1, x2, z2)
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