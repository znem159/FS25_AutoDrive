ADCollSensorSplit = ADInheritsFrom(ADSensor)

function ADCollSensorSplit:new(vehicle, sensorParameters)
    local o = ADCollSensorSplit:create()
    o:init(vehicle, ADSensor.TYPE_COLLISION, sensorParameters)
    o.hit = false
    o.newHit = false
    o.collisionHits = 0
    o.timeOut = AutoDriveTON:new()
    o.vehicle = vehicle;

    o.mask = 0

    return o
end

function ADCollSensorSplit.getMask()
    return CollisionFlag.DEFAULT + CollisionFlag.STATIC_OBJECT + CollisionFlag.DYNAMIC_OBJECT + CollisionFlag.VEHICLE + CollisionFlag.TERRAIN_DELTA + CollisionFlag.TREE + CollisionFlag.BUILDING 
end

function ADCollSensorSplit:onUpdate(dt)
    self.mask = self:getMask()
    -- Here i want to generate an array of boxes instead of a large rotated single one

    -- Old
    --  |--------|     /\
    --  |Vehicle-|    /  \
    --  |--------|   /    \
    --               \     \
    --                \    /
    --                 \  /
    --                  \/
    --

     -- New
    --  |--------|/\
    --  |Vehicle-|/\\
    --  |--------|/\\\
    --            \ \\\
    --             \ \\\
    --              \ \/
    --               \/

    local boxes = self:getBoxShapes()

    --if self.collisionHits == 0 or self.timeOut:timer(true, 20000, dt) then
        self.timeOut:timer(false)
        self.hit = self.newHit
        self:setTriggered(self.hit)
        self.newHit = false

        for _, box in pairs(boxes) do
            local offsetCompensation = math.max(-math.tan(box.rx) * box.size[3], 0)
            box.y = math.max(getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, box.x, 300, box.z), box.y) + offsetCompensation

            self.collisionHits = overlapBox(box.x, box.y, box.z, box.rx, box.ry, 0, box.size[1], box.size[2], box.size[3], "collisionTestCallback", self, self.mask, true, true, true, true) --AIVehicleUtil.COLLISION_MASK --16783599
        end
        --for some reason, I have to call this again if collisionHits > 0 to trigger the callback functions, which check if the hit object is me or is attached to me
        --if self.collisionHits > 0 then
            --overlapBox(box.x, box.y, box.z, box.rx, box.ry, 0, box.size[1], box.size[2], box.size[3], "collisionTestCallback", self, self.mask, true, true, true)
        --end
    --end

    self:onDrawDebug(boxes)
end

function ADCollSensorSplit:collisionTestCallback(transformId)
    self.collisionHits = math.max(0, self.collisionHits - 1)
    local unloadDriver = ADHarvestManager:getAssignedUnloader(self.vehicle)
    local collisionObject = g_currentMission.nodeToObject[transformId]

    if collisionObject == nil then
        -- let try if parent is a object
        local parent = getParent(transformId)
        if parent then
            collisionObject = g_currentMission.nodeToObject[parent]
        end
    end

    if collisionObject ~= nil then
        if collisionObject ~= self and collisionObject ~= self.vehicle and not AutoDrive:checkIsConnected(self.vehicle:getRootVehicle(), collisionObject) then
            if unloadDriver == nil or (collisionObject ~= unloadDriver and (not AutoDrive:checkIsConnected(unloadDriver:getRootVehicle(), collisionObject))) then
                self.newHit = true
                return true
            end
        end
    else
        self.newHit = true
        return true
    end

    return false
end

function ADCollSensorSplit:buildBoxShape(x, y, z, width, height, length, vecZ, vecX)
    local vehicle = self.vehicle

    local box = {}
    box.offset = {}
    box.size = {}
    box.center = {}
    box.size[1] = width * 0.5
    box.size[2] = height * 0.5
    box.size[3] = length * 0.5
    box.offset[1] = x
    box.offset[2] = y
    box.offset[3] = z
    box.center[1] = box.offset[1] + vecZ.x * box.size[3]
    box.center[2] = box.offset[2] + box.size[2]
    box.center[3] = box.offset[3] + vecZ.z * box.size[3]

    box.topLeft = {}
    box.topLeft[1] = box.center[1] - vecX.x * box.size[1] + vecZ.x * box.size[3]
    box.topLeft[2] = box.center[2]
    box.topLeft[3] = box.center[3] - vecX.z * box.size[1] + vecZ.z * box.size[3]

    box.topRight = {}
    box.topRight[1] = box.center[1] + vecX.x * box.size[1] + vecZ.x * box.size[3]
    box.topRight[2] = box.center[2]
    box.topRight[3] = box.center[3] + vecX.z * box.size[1] + vecZ.z * box.size[3]

    box.downRight = {}
    box.downRight[1] = box.center[1] + vecX.x * box.size[1] - vecZ.x * box.size[3]
    box.downRight[2] = box.center[2]
    box.downRight[3] = box.center[3] + vecX.z * box.size[1] - vecZ.z * box.size[3]

    box.downLeft = {}
    box.downLeft[1] = box.center[1] - vecX.x * box.size[1] - vecZ.x * box.size[3]
    box.downLeft[2] = box.center[2]
    box.downLeft[3] = box.center[3] - vecX.z * box.size[1] - vecZ.z * box.size[3]

    box.dirX, box.dirY, box.dirZ = localDirectionToWorld(vehicle.components[1].node, 0, 0, 1)
    box.zx, box.zy, box.zz = localDirectionToWorld(vehicle.components[1].node, vecZ.x, 0, vecZ.z)
    
    box.ry = math.atan2(box.zx, box.zz)

    local angleOffset = 4
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    if not AutoDrive.checkIsOnField(x, y, z) and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule:isActive() then
        local heightDiff = self.vehicle.ad.drivePathModule:getApproachingHeightDiff()
        if heightDiff < 1.5 and heightDiff > -1 then
            angleOffset = 0
        end
    end
    box.rx = -MathUtil.getYRotationFromDirection(box.dirY, 1) * self.frontFactor - math.rad(angleOffset)
    box.x, box.y, box.z = localToWorld(vehicle.components[1].node, box.center[1], box.center[2], box.center[3])

    box.topLeft.x, box.topLeft.y, box.topLeft.z = localToWorld(vehicle.components[1].node, box.topLeft[1], box.topLeft[2], box.topLeft[3])
    box.topRight.x, box.topRight.y, box.topRight.z = localToWorld(vehicle.components[1].node, box.topRight[1], box.topRight[2], box.topRight[3])
    box.downRight.x, box.downRight.y, box.downRight.z = localToWorld(vehicle.components[1].node, box.downRight[1], box.downRight[2], box.downRight[3])
    box.downLeft.x, box.downLeft.y, box.downLeft.z = localToWorld(vehicle.components[1].node, box.downLeft[1], box.downLeft[2], box.downLeft[3])

    return box
end

function ADCollSensorSplit:getBoxShapes()
    local vehicle = self.vehicle

    local width, length = AutoDrive.getVehicleDimensions(vehicle, false)

    local frontCenterPoint = {}
    frontCenterPoint.x, frontCenterPoint.y, frontCenterPoint.z = worldToLocal(self.vehicle.components[1].node, 0, 0, AutoDrive.getVehicleLeadingEdge(self.vehicle))
    
    local lookAheadDistance = math.clamp(vehicle.lastSpeedReal * 3600 * 15.5 / 40, self.minDynamicLength, 16)
    
    local vecZ = {x = math.sin(vehicle.rotatedTime), z = math.cos(vehicle.rotatedTime)}
    local vecX = {x = vecZ.z, z = -vecZ.x}

    local boxYPos = AutoDrive.getSetting("collisionHeigth", self.vehicle) or 2
    local boxHeight = 0.75

    local numberOfBoxes = 5
    local boxWidth = width / numberOfBoxes
    local boxes = {}
    for i=1, numberOfBoxes do
        local xOffset = (-width / 2) + (i - 0.5) * boxWidth
        boxes[i] = self:buildBoxShape(
            self.location.x + xOffset, boxYPos, self.location.z,
            boxWidth, boxHeight, lookAheadDistance,
            vecZ, vecX
        )
    end

    return boxes
end

function ADCollSensorSplit:onDrawDebug(boxes)
    if self.drawDebug or AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
        local red = 1
        local green = 0
        local blue = 0
        local isTriggered = self:isTriggered()
        if isTriggered then
            if self.sensorType == ADSensor.TYPE_FRUIT then
                blue = 1
            end
            if self.sensorType == ADSensor.TYPE_FIELDBORDER then
                green = 1
            end
        end
        
        for _, box in pairs(boxes) do
            if isTriggered then
                DebugUtil.drawOverlapBox(box.x, box.y, box.z, box.rx, box.ry, 0, box.size[1], box.size[2], box.size[3], 1, 0, 0)
            else
                DebugUtil.drawOverlapBox(box.x, box.y, box.z, box.rx, box.ry, 0, box.size[1], box.size[2], box.size[3], 1, 1, 1)
            end
        end
    end
end