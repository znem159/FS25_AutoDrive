--[[
New to Pathfinder:
- 3 Settings are considered: restrictToField, avoidFruit, pathFinderTime

1. restrictToField
The Pathfinder tries to find a path to target in the limit of the current field borders.
This is only possible if the vehicle is located inside a field.
If disabled, only setting avoidFruit limits or not, the shortest path to target will be calculated!

2. avoidFruit
The Pathfinder tries to find a path to target without passing through fruit.
This is effective if vehicle is inside a field.
NEW: Working also outside of a field, i.e. if possible a path around a field to network will be searched for!
If disabled, the shortest path to target will be calculated!

3. pathFinderTime
Is more a factor for the range the Pathfinder will search for path to target, default is 1.
In case fruit avoid on large fields with sufficient headland is not working, you may try to increase.
But be aware the calculation time will increase as well!

- 3 fallback scenario will automatic take effect:
If setting restrictToField is enabled, fallback 1 and 2 are possible:
fallback 1:
The first fallback will extend the field border by 30+ meters around the field pathfinding was started.
With this the vehicle will search for a path also around the field border!
30+ meters depend on the vehicle + trailer turn radius, if greater then extended wider.
fallback 2:
Second fallback will deactivate all field border restrictions, means setting avoidFruit will limit or not the search range for path.

fallback 3:
Third fallback will take effect only if setting avoidFruit is enabled.
It will disable fruit avoid automatically if no path was found.

Inside informations:
This is a calculation with the worst assumption of all cells to be checked:

Number of cells:
#cells = MAX_PATHFINDER_STEPS_PER_FRAME / 2 * MAX_PATHFINDER_STEPS_TOTAL * 3 (next directions - see determineNextGridCells)

PathFinderModule.MAX_PATHFINDER_STEPS_PER_FRAME = 10
PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL = 400
#cells = 6000

with minTurnRadius = 7m calculated area:

cellsize = 7m * 7m = 49m^2
overall area = #cells * cellsize * pathFinderTime

with pathFinderTime = 1:
overall area = 6000 * 49 * 1 = 294000 m^2
for quadrat field layout: side length ~ 540m

with pathFinderTime = 2: side length ~ 760m
with pathFinderTime = 3: side length ~ 940m

This is inclusive of the field border cells!
]]

PathFinderModule = {}
PathFinderModule.debug = false

PathFinderModule.PATHFINDER_MAX_RETRIES = 3
PathFinderModule.MAX_PATHFINDER_STEPS_PER_FRAME = 2
PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL = 400
PathFinderModule.MAX_PATHFINDER_STEPS_COMBINE_TURN = 100
PathFinderModule.PATHFINDER_FOLLOW_DISTANCE = 45
PathFinderModule.PATHFINDER_TARGET_DISTANCE = 7
PathFinderModule.PATHFINDER_TARGET_DISTANCE_PIPE = 16
PathFinderModule.PATHFINDER_TARGET_DISTANCE_PIPE_CLOSE = 6
PathFinderModule.PATHFINDER_START_DISTANCE = 7
PathFinderModule.MAX_FIELDBORDER_CELLS = 5
PathFinderModule.PATHFINDER_MIN_DISTANCE_START_TARGET = 50

PathFinderModule.PP_MIN_DISTANCE = 20
PathFinderModule.PP_CELL_X = 9
PathFinderModule.PP_CELL_Z = 9

PathFinderModule.GRID_SIZE_FACTOR = 0.5
PathFinderModule.GRID_SIZE_FACTOR_SECOND_UNLOADER = 1.1

PathFinderModule.MIN_FRUIT_VALUE = 50
PathFinderModule.SLOPE_DETECTION_THRESHOLD = math.rad(20)
PathFinderModule.NEW_PF_STEP_FACTOR = 4

PathFinderModule.NO_FALLBACK = 0
PathFinderModule.FALLBACK_OFF_FIELD = 1
PathFinderModule.FALLBACK_OFF_FIELD_BORDER = 2
PathFinderModule.FALLBACK_FRUIT = 3

--[[

The PathFinderModule manages the high level stuff for Pathfinding, and delegates the actual path finding to the selected 
path finder.
This path finder needs to implement these interface methods:

Issue new pathfinder task to:
startPathPlannningTo (pathfinderTask) : nil

Work on task:
update(dt) : nil 

hasFinished() : Bool
isBlocked() : Bool

Return ordered list of waypoints (including optional appendedWaypoint of task)
getPath() : table of waypoints

--]]

function PathFinderModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    o.legacy = PathFinderLegacy:new(vehicle)
    o.aStarDubins = PathFinderAStarDubins:new(vehicle)
    o.treeCrawler = PathFinderTreeCrawler:new(vehicle)
    o.currentPathfinder = o.aStarDubins
    PathFinderModule.reset(o)
    return o
end

function PathFinderModule:reset()
    PathFinderModule.debugMsg(self.vehicle, "PFM:reset start")
    self.retryCounter = 0
    self.goingToNetwork = false
    self.fallBackMode = PathFinderModule.NO_FALLBACK
    self.destinationId = nil
    self.appendWayPoints = {}
    self.inProgress = false
    self.delayTime = 0
    
    self.legacy:reset()
    self.aStarDubins:reset()
end

function PathFinderModule:hasFinished()
    return (not self.inProgress) or self.currentPathfinder:hasFinished()
end

function PathFinderModule:isBlocked()
    return self.currentPathfinder:isBlocked()
end

function PathFinderModule:getPath()
    return self.currentPathfinder:getPath()
end

function PathFinderModule:startPathPlanningToNetwork(destinationId)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningToNetwork destinationId %s"
        , tostring(destinationId)
    )
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningToNetwork destinationId %s",
            tostring(destinationId)
        )
    )
    local closest = self.vehicle:getClosestWayPoint()
    self.goingToNetwork = true
    self:startPathPlanningToWayPoint(closest, destinationId)
end

function PathFinderModule:startPathPlanningToWayPointWithTargetVector(wayPointId, targetVector)    
    local targetNode = ADGraphManager:getWayPointById(wayPointId)
    self.appendWayPoints = {}

    local pathfinderTask = {
        targetPoint = targetNode,
        targetVector = targetVector,
        toNetwork = true,
        toPipe = false,
        fruitToCheckFor = nil,
        wayPointsToAppend = self.appendWayPoints,
        fallBackMode = PathFinderModule.NO_FALLBACK,
        chasingVehicle = false,
        isSecondChasingVehicle = false
    }

    self:startPathPlanningTo(pathfinderTask)
end

function PathFinderModule:startPathPlanningToWayPoint(wayPointId, destinationId)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningToWayPoint destinationId %s"
        , tostring(destinationId)
    )
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningToWayPoint wayPointId %s",
            tostring(wayPointId)
        )
    )
    local targetNode = ADGraphManager:getWayPointById(wayPointId)
    local wayPoints = ADGraphManager:pathFromTo(wayPointId, destinationId)
    if wayPoints ~= nil and #wayPoints > 1 then
        local vecToNextPoint = {x = wayPoints[2].x - targetNode.x, z = wayPoints[2].z - targetNode.z}
        self.goingToNetwork = true
        self.destinationId = destinationId
        self.targetWayPointId = wayPointId
        self.appendWayPoints = wayPoints

        local pathfinderTask = {
            targetPoint = targetNode,
            targetVector = vecToNextPoint,
            toNetwork = true,
            toPipe = false,
            fruitToCheckFor = nil,
            wayPointsToAppend = self.appendWayPoints,
            fallBackMode = PathFinderModule.NO_FALLBACK,
            chasingVehicle = false,
            isSecondChasingVehicle = false
        }

        self:startPathPlanningTo(pathfinderTask)
    end
end

function PathFinderModule:startPathPlanningToPipe(combine, chasing)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningToPipe chasing %s"
        , tostring(chasing)
    )
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:startPathPlanningToPipe")
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningToPipe combine %s",
            tostring(combine:getName())
        )
    )
    self.appendWayPoints = {}
    local _, worldY, _ = getWorldTranslation(combine.components[1].node)
    local rx, _, rz = localDirectionToWorld(combine.components[1].node, 0, 0, 1)
    if combine.components[2] ~= nil and combine.components[2].node ~= nil then
        rx, _, rz = localDirectionToWorld(combine.components[2].node, 0, 0, 1)
    end
    local combineVector = {x = rx, z = rz}

    local pipeChasePos, pipeChaseSide = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeChasePosition(true)
    -- We use the follow distance as a proxy measure for "what works" for the size of the
    -- field being worked.
    -- local followDistance = AutoDrive.getSetting("followDistance", self.vehicle)
    -- Use the length of the tractor-trailer combo to determine how far to drive to straighten
    -- the trailer.
    -- 2*math.sin(math.pi/8)) is the third side of a 45-67.5-67.5 isosceles triangle with the
    -- equal sides being the length of the tractor train
    local lengthOffset = combine.size.length / 2 +
                            AutoDrive.getTractorTrainLength(self.vehicle, true, false) * (2 * math.sin(math.pi / 8))   
    if lengthOffset <= self.PATHFINDER_TARGET_DISTANCE then
        lengthOffset = self.PATHFINDER_TARGET_DISTANCE
    end   

    local fruitToCheckFor = nil
    if combine.spec_combine ~= nil and combine.ad.isHarvester then
        if combine.spec_combine.fillUnitIndex ~= nil and combine.spec_combine.fillUnitIndex ~= 0 then
            local fillType = g_fruitTypeManager:getFruitTypeIndexByFillTypeIndex(combine:getFillUnitFillType(combine.spec_combine.fillUnitIndex))
            if fillType ~= nil then
                fruitToCheckFor = fillType
                local fruitType = g_fruitTypeManager:getFruitTypeByIndex(fillType)

                PathFinderModule.debugVehicleMsg(self.vehicle,
                    string.format("PFM startPathPlanningToPipe self.fruitToCheck %s Fruit name %s title %s",
                        tostring(fruitToCheckFor),
                        tostring(fruitType.fillType.name),
                        tostring(fruitType.fillType.title)
                    )
                )
            end
        end
    end

    if combine.ad.isAutoAimingChopper then
        local pathFinderTarget = {x = pipeChasePos.x, y = worldY, z = pipeChasePos.z}

        local pathfinderTask = {
            targetPoint = pathFinderTarget,
            targetVector = combineVector,
            toNetwork = false,
            toPipe = true,
            fruitToCheckFor = fruitToCheckFor,
            wayPointsToAppend = self.appendWayPoints,
            fallBackMode = PathFinderModule.NO_FALLBACK,
            chasingVehicle = chasing,
            isSecondChasingVehicle = false
        }

        self:startPathPlanningTo(pathfinderTask)
    elseif combine.ad.isFixedPipeChopper then
        local pathFinderTarget = {x = pipeChasePos.x, y = worldY, z = pipeChasePos.z}
        -- only append target points / try to straighten the driver/trailer combination if we are driving up to the pipe not the rear end
        if pipeChaseSide ~= AutoDrive.CHASEPOS_REAR then
            pathFinderTarget = {x = pipeChasePos.x - (combine.size.length) * rx, y = worldY, z = pipeChasePos.z - (combine.size.length) * rz}
        end
        local appendedNode = {x = pipeChasePos.x - (combine.size.length / 2 * rx), y = worldY, z = pipeChasePos.z - (combine.size.length / 2 * rz)}
        
        if pipeChaseSide ~= AutoDrive.CHASEPOS_REAR then
            table.insert(self.appendWayPoints, appendedNode)
        end
        
        local pathfinderTask = {
            targetPoint = pathFinderTarget,
            targetVector = combineVector,
            toNetwork = false,
            toPipe = true,
            fruitToCheckFor = fruitToCheckFor,
            wayPointsToAppend = self.appendWayPoints,
            fallBackMode = PathFinderModule.NO_FALLBACK,
            chasingVehicle = chasing,
            isSecondChasingVehicle = false
        }

        self:startPathPlanningTo(pathfinderTask)        
    else
        -- combine.ad.isHarvester 
        local pathFinderTarget = {x = pipeChasePos.x, y = worldY, z = pipeChasePos.z}
        -- only append target points / try to straighten the driver/trailer combination if we are driving up to the pipe not the rear end
        if pipeChaseSide ~= AutoDrive.CHASEPOS_REAR then
            pathFinderTarget = {x = pipeChasePos.x - (lengthOffset) * rx, y = worldY, z = pipeChasePos.z - (lengthOffset) * rz}
        end
        local appendedNode = {x = pipeChasePos.x - (combine.size.length / 2 * rx), y = worldY, z = pipeChasePos.z - (combine.size.length / 2 * rz)}

        if pipeChaseSide ~= AutoDrive.CHASEPOS_REAR then
            table.insert(self.appendWayPoints, appendedNode)
            table.insert(self.appendWayPoints, pipeChasePos)
        end

        local pathfinderTask = {
            targetPoint = pathFinderTarget,
            targetVector = combineVector,
            toNetwork = false,
            toPipe = true,
            fruitToCheckFor = fruitToCheckFor,
            wayPointsToAppend = self.appendWayPoints,
            fallBackMode = PathFinderModule.NO_FALLBACK,
            chasingVehicle = chasing,
            isSecondChasingVehicle = false
        }

        self:startPathPlanningTo(pathfinderTask)        
    end    
end

function PathFinderModule:startPathPlanningToVehicle(targetVehicle, targetDistance)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningToVehicle targetDistance %s"
        , tostring(targetDistance)
    )
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningToVehicle targetVehicle %s",
            tostring(targetVehicle:getName())
        )
    )
    local worldX, worldY, worldZ = getWorldTranslation(targetVehicle.components[1].node)
    local rx, _, rz = localDirectionToWorld(targetVehicle.components[1].node, 0, 0, 1)
    local targetVector = {x = rx, z = rz}

    local wpBehind = {x = worldX - targetDistance * rx, y = worldY, z = worldZ - targetDistance * rz}

    local pathfinderTask = {
        targetPoint = wpBehind,
        targetVector = targetVector,
        toNetwork = false,
        toPipe = false,
        fruitToCheckFor = nil,
        wayPointsToAppend = {},
        fallBackMode = PathFinderModule.NO_FALLBACK,
        chasingVehicle = true,
        isSecondChasingVehicle = true
    }

    self:startPathPlanningTo(pathfinderTask)
end

function PathFinderModule:startPathPlanningTo(pathfinderTask)
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningTo targetPoint x,z %d %d",
            math.floor(pathfinderTask.targetPoint.x),
            math.floor(pathfinderTask.targetPoint.z)
        )
    )
    ADScheduler:addPathfinderVehicle(self.vehicle)
    if math.abs(pathfinderTask.targetVector.x) < 0.001 then
        pathfinderTask.targetVector.x = 0.001
    end
    if math.abs(pathfinderTask.targetVector.z) < 0.001 then
        pathfinderTask.targetVector.z = 0.001
    end
    
    --targetPoint, targetVector, toNetwork, toPipe, fruitToCheckFor, wayPointsToAppend, fallBackMode, chasingVehicle, isSecondChasingVehicle

    self.inProgress = true
    self.currentTask = pathfinderTask

    local angleRad = AutoDrive.normalizeAngle(math.atan2(self.currentTask.targetVector.z, self.currentTask.targetVector.x))
    local vehicleWorldX, vehicleWorldY, vehicleWorldZ = getWorldTranslation(self.vehicle.components[1].node)
    local targetX = self.currentTask.targetPoint.x - math.cos(angleRad) * PathFinderModule.PATHFINDER_TARGET_DISTANCE
    local targetZ = self.currentTask.targetPoint.z - math.sin(angleRad) * PathFinderModule.PATHFINDER_TARGET_DISTANCE

    self.startIsOnField = AutoDrive.checkIsOnField(vehicleWorldX, vehicleWorldY, vehicleWorldZ) and self.vehicle.ad.sensors.frontSensorField:pollInfo(true)
    self.endIsOnField = AutoDrive.checkIsOnField(targetX, vehicleWorldY, targetZ)
    self.restrictToField = AutoDrive.getSetting("restrictToField", self.vehicle) and self.startIsOnField and self.endIsOnField
    self.avoidFruitSetting = AutoDrive.getSetting("avoidFruit", self.vehicle)

    if AutoDrive.getSetting("Pathfinder") == 1 then
        self.currentPathfinder = self.aStarDubins
    elseif AutoDrive.getSetting("Pathfinder") == 0 then
        self.currentPathfinder = self.legacy
    elseif AutoDrive.getSetting("Pathfinder") == 2 then
        self.currentPathfinder = self.treeCrawler
    end

    self.currentPathfinder:startPathPlanningTo(self.currentTask)
end

function PathFinderModule:restartAtNextWayPoint()
    self.targetWayPointId = self.appendWayPoints[2].id
    local targetNode = ADGraphManager:getWayPointById(self.targetWayPointId)
    local wayPoints = ADGraphManager:pathFromTo(self.targetWayPointId, self.destinationId)
    if wayPoints ~= nil and #wayPoints > 1 then
        local vecToNextPoint = {x = wayPoints[2].x - targetNode.x, z = wayPoints[2].z - targetNode.z}
        self.fallBackMode = PathFinderModule.NO_FALLBACK
        self.appendWayPoints = wayPoints

        self.currentTask.targetPoint = targetNode
        self.currentTask.targetVector = vecToNextPoint

        self:startPathPlanningTo(self.currentTask)
    else
        self:abort()
    end
end

function PathFinderModule:abort()
    PathFinderModule.debugMsg(self.vehicle, "PFM:abort start")
    self.inProgress = false

    ADScheduler:removePathfinderVehicle(self.vehicle)
end

-- return the actual and max number of iterations the pathfinder will perform by itself, could be used to show info in HUD
function PathFinderModule:getCurrentState()
    local maxStates = 1
    local actualState = 1
    if self.restrictToField then
        maxStates = maxStates + 2
    end
    if self.avoidFruitSetting then
        maxStates = maxStates + 1
    end
    if self.destinationId ~= nil then
        maxStates = maxStates + PathFinderModule.PATHFINDER_MAX_RETRIES
    end

    actualState = self.fallBackMode
    
    if self.destinationId ~= nil then
        actualState = actualState + self.retryCounter
    end

    -- TODO: Get this from the currentPathfinder
    return actualState, maxStates, 1, 1
end

function PathFinderModule:update(dt)
    self.delayTime = math.max(0, self.delayTime - dt)
    if self.delayTime > 0 then
        return
    end
    
    self.currentPathfinder:update(dt)

    if self.currentPathfinder:isBlocked() then        
        -- Only allow fallback if we are not heading for a moving vehicle
        local fallbacksAllowed = not self.chasingVehicle

        -- Only allow auto restart when planning path to network and we can adjust target wayPoint
        local retryAllowed = self.destinationId ~= nil and self.retryCounter < self.PATHFINDER_MAX_RETRIES

        if self.fallBackMode == PathFinderModule.NO_FALLBACK and fallbacksAllowed and self.restrictToField then
            self.fallBackMode = PathFinderModule.FALLBACK_OFF_FIELD
            self.currentTask.fallBackMode = self.fallBackMode
            self:startPathPlanningTo(self.currentTask)
        elseif self.fallBackMode == PathFinderModule.FALLBACK_OFF_FIELD and fallbacksAllowed and self.restrictToField and not AutoDrive.getSetting("Pathfinder") == 1 then            
            self.fallBackMode = PathFinderModule.FALLBACK_OFF_FIELD_BORDER
            self.currentTask.fallBackMode = self.fallBackMode
            self:startPathPlanningTo(self.currentTask)
        elseif self.fallBackMode == PathFinderModule.FALLBACK_OFF_FIELD_BORDER and fallbacksAllowed and self.avoidFruitSetting then
            self.fallBackMode = PathFinderModule.FALLBACK_FRUIT
            self.currentTask.fallBackMode = self.fallBackMode
            self:startPathPlanningTo(self.currentTask)
        elseif retryAllowed then
            self.retryCounter = self.retryCounter + 1
            --if we are going to the network and can't find a path. Just select the next waypoint for now
            if self.appendWayPoints ~= nil and #self.appendWayPoints > 2 then                
                self:restartAtNextWayPoint()
            else
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - error - retryAllowed: yes -> but no appendWayPoints")
                self:abort()
            end
        else
            if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                return
            end
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - error - retryAllowed: no -> fallBackModeAllowed: no -> aborting now")
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM update - error - retryAllowed: no -> fallBackModeAllowed: no -> aborting now"
                )
            )
            self:abort()
        end
    end
end

function PathFinderModule:addDelayTimer(delayTime)      -- used in: ExitFieldTask, UnloadAtDestinationTask, CatchCombinePipeTask, EmptyHarvesterTask
    self.delayTime = delayTime
end

function PathFinderModule.debugVehicleMsg(vehicle, msg)
    -- collect output for single vehicle - help to examine sequences for a single vehicle
    if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.debug ~= nil then
        if AutoDrive.debugVehicleMsg ~= nil then
            AutoDrive.debugVehicleMsg(vehicle, msg)
        end
    end
end

function PathFinderModule.debugMsg(vehicle, debugText, ...)
    if PathFinderModule.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_PATHINFO, debugText, ...)
    end
end
