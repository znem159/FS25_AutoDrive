AutoDrivePlaceableData = {}

function AutoDrivePlaceableData:load()
    AutoDrivePlaceableData.active = false
	AutoDrivePlaceableData.reset()
end

function AutoDrivePlaceableData.reset()
	if AutoDrivePlaceableData.xmlFile then
		delete(AutoDrivePlaceableData.xmlFile)
	end
    AutoDrivePlaceableData.placeable = nil
    AutoDrivePlaceableData.xmlFile = nil
	AutoDrivePlaceableData.wayPoints = {}
	AutoDrivePlaceableData.mapMarkers = {}
    AutoDrivePlaceableData.showErrorDialog = nil
    AutoDrivePlaceableData.showConfirmationDialog = nil
end

function AutoDrivePlaceableData.prerequisitesPresent(specializations)
    return true
end

function AutoDrivePlaceableData.registerEventListeners(placeableType)
    for _, n in pairs(
        {
            "onFinalizePlacement"
        }
    ) do
        SpecializationUtil.registerEventListener(placeableType, n, AutoDrivePlaceableData)
    end
end

function AutoDrivePlaceableData:setActive(active)
	if active ~= nil then
		AutoDrivePlaceableData.active = active
	end
end

function AutoDrivePlaceableData.callBack(result)
	if AutoDrivePlaceableData.showConfirmationDialog then
		AutoDrivePlaceableData.showConfirmationDialog = nil
		if result == true then
			if AutoDrivePlaceableData.xmlFile then
				local ret = AutoDrivePlaceableData.readGraphFromXml(AutoDrivePlaceableData.xmlFile, AutoDrivePlaceableData.placeable)
				if ret < 0 then
					AutoDrivePlaceableData.showError(ret)
				else
					AutoDrivePlaceableData.reset() -- all done
				end
			end
		end
	end
	if AutoDrivePlaceableData.showErrorDialog then
		AutoDrivePlaceableData.showErrorDialog = nil
		AutoDrivePlaceableData.reset() -- all done
	end

end

function AutoDrivePlaceableData.showError(error)
	local args = {text = "AutoDrive: Network inconsistent, error code " .. error}
	local dialog = g_gui:showDialog("InfoDialog")
	if dialog then
		dialog.target:setDialogType(Utils.getNoNil(args.dialogType, DialogElement.TYPE_WARNING))
		dialog.target:setText(args.text)
		dialog.target:setCallback(AutoDrivePlaceableData.callBack)
		-- dialog.target:setButtonTexts(args.okText)
		-- dialog.target:setButtonAction(args.buttonAction)
		AutoDrivePlaceableData.showErrorDialog = dialog
	end
end

function AutoDrivePlaceableData.showConfirmation()
	local args = {text = "AutoDrive: Would you like to import the Network from the Placeable "}
	local dialog = g_gui:showDialog("YesNoDialog")
	if dialog then
		dialog.target:setDialogType(Utils.getNoNil(args.dialogType, DialogElement.TYPE_QUESTION))
		dialog.target:setText(args.text)
		dialog.target:setCallback(AutoDrivePlaceableData.callBack)
		-- dialog.target:setButtonTexts(args.okText)
		-- dialog.target:setButtonAction(args.buttonAction)
		AutoDrivePlaceableData.showConfirmationDialog = dialog
	end
end

function AutoDrivePlaceableData:onFinalizePlacement()
    if AutoDrivePlaceableData.active then
        if g_currentMission.placeableSystem and g_currentMission.placeableSystem.placeables and #g_currentMission.placeableSystem.placeables > 0 then
			AutoDrivePlaceableData.placeable = g_currentMission.placeableSystem.placeables[#g_currentMission.placeableSystem.placeables]
			local xmlFileName = g_currentMission.placeableSystem.placeables[#g_currentMission.placeableSystem.placeables].configFileName
            if xmlFileName then
                AutoDrivePlaceableData.xmlFile = loadXMLFile("placeable_xml", xmlFileName)
				if AutoDrivePlaceableData.xmlFile then
					if hasXMLProperty(AutoDrivePlaceableData.xmlFile, "placeable")
					and hasXMLProperty(AutoDrivePlaceableData.xmlFile, "placeable.AutoDrive")
					and hasXMLProperty(AutoDrivePlaceableData.xmlFile, "placeable.AutoDrive.wayPoints")
					then
						AutoDrivePlaceableData.showConfirmation()
					else
						-- reset all
						AutoDrivePlaceableData.reset()
					end
				else
					-- reset all
					AutoDrivePlaceableData.reset()
				end
            end
		end
    end
end


function AutoDrivePlaceableData.readGraphFromXml(xmlFile, placeable)
	AutoDrivePlaceableData.wayPoints = {}
	AutoDrivePlaceableData.mapMarkers = {}
	local px, py, pz = placeable:getPosition()

	do
		local function checkProperty(key)
			if not hasXMLProperty(xmlFile, key) then
				return -1
			end
		end

		local function checkString(key)
			local tempString = getXMLString(xmlFile, key)
			if tempString == nil then
				return -2
			end
			local xTable = tempString:split(",")
			if #xTable == 0 then
				return -3
			end
		end

		checkProperty("placeable")
		checkProperty("placeable.AutoDrive")
		checkProperty("placeable.AutoDrive.wayPoints")

		local key
		local tempString

		key = "placeable.AutoDrive.waypoints.x"
		checkProperty(key)
		checkString(key)
		local xt = getXMLString(xmlFile, key):split(",")
		key = "placeable.AutoDrive.waypoints.y" -- not required, only for consitency check
		checkProperty(key)
		checkString(key)
		local yt = getXMLString(xmlFile, key):split(",")
		key = "placeable.AutoDrive.waypoints.z"
		checkProperty(key)
		checkString(key)
		local zt = getXMLString(xmlFile, key):split(",")
		key = "placeable.AutoDrive.waypoints.out"
		checkProperty(key)
		-- checkString(key)
		local ot = getXMLString(xmlFile, key):split(";")
		key = "placeable.AutoDrive.waypoints.incoming"
		checkProperty(key)
		-- checkString(key)
		local it = getXMLString(xmlFile, key):split(";")
		key = "placeable.AutoDrive.waypoints.flags"
		checkProperty(key)
		checkString(key)
		local ft = getXMLString(xmlFile, key):split(",")

        if #xt == 0 or #yt == 0 or #zt == 0 or #ot == 0 or #it == 0 or #ft == 0 or #xt ~= #yt or #xt ~= #zt or #xt ~= #ot or #xt ~= #it or #xt ~= #ft then
			AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency key %s"
			, tostring(key)
			)
			if #xt == 0 then
				AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt == 0")
			end
			if #yt == 0 then
				AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #yt == 0")
			end
			if #zt == 0 then
				AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #zt == 0")
			end
			if #ot == 0 then
				AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #ot == 0")
			end
			if #it == 0 then
				AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #it == 0")
			end
			if #ft == 0 then
				AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #ft == 0")
			end

			if #xt ~= #yt then
				AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt ~= #yt")
			end
			if #xt ~= #zt then
				AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt ~= #zt")
			end
			if #xt ~= #ot then
				AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt ~= #ot")
			end
			if #xt ~= #it then
				AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt ~= #it")
			end
			if #xt ~= #ft then
				AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt ~= #ft %s %s"
				, tostring(#xt)
				, tostring(#ft)
				)
			end

			return -4
        end

        local waypointsCount = #xt

        local mapMarker = {}
        local mapMarkerCounter = 1

		while mapMarker ~= nil and mapMarkerCounter < 10 do
            mapMarker.id = getXMLFloat(xmlFile, "placeable.AutoDrive.mapmarker.mm" .. mapMarkerCounter .. ".id")
            -- if id is still nil, we are at the end of the list and stop here

			if mapMarker.id == nil then
                mapMarker = nil
                break
            end
            if mapMarker.id > waypointsCount then -- invalid marker id
                return -5
            end

			mapMarker.id = ADGraphManager:getWayPointsCount() + mapMarkerCounter

			mapMarker.name = getXMLString(xmlFile, "placeable.AutoDrive.mapmarker.mm" .. mapMarkerCounter .. ".name")
            if mapMarker.name then
                mapMarker.name = (#ADGraphManager:getMapMarkers() + 1) .. "_" .. mapMarker.name -- add the # in front to avoid multiple same names
			else
				return -6
            end
            mapMarker.group = "All"
            -- make sure group existst
            if ADGraphManager:getGroupByName(mapMarker.group) == nil then
                ADGraphManager:addGroup(mapMarker.group)
            end
			table.insert(AutoDrivePlaceableData.mapMarkers, mapMarker) -- collect all mapMarkers to be created, but do it only at end if everthing is fine
            mapMarker = {}
            mapMarkerCounter = mapMarkerCounter + 1
        end
        -- done loading Map Markers

        -- localization for better performances
		local tnum = tonumber
		local tbin = table.insert
		local stsp = string.split

		for i = 1, waypointsCount do
			local wpx = tnum(xt[i]) + px
			local wpz = tnum(zt[i]) + pz
			local wp = {
				id = i,
				x = wpx,
				-- y = tnum(yt[i]), -- not required, will be adjusted to terrain
				y = AutoDrive:getTerrainHeightAtWorldPos(wpx, wpz) + 2,
				z = wpz,
				out = {}, 
				incoming = {}
			}
			if ot[i] and ot[i] ~= "-1" then
				for _, out in pairs(stsp(ot[i], ",")) do
					-- tbin(wp.out, tnum(out))
					local num = tnum(out) and tnum(out) or 0
					if num > 0 and num <= waypointsCount then
						-- avoid inconsistent links
						tbin(wp.out, num + ADGraphManager:getWayPointsCount())
					end
				end
			end
			if it[i] and it[i] ~= "-1" then
				for _, incoming in pairs(stsp(it[i], ",")) do
					-- tbin(wp.incoming, tnum(incoming))
					local num = tnum(incoming) and tnum(incoming) or 0
					if num > 0 and num <= waypointsCount then
						-- avoid inconsistent links
						tbin(wp.incoming, num + ADGraphManager:getWayPointsCount())
					end
				end
			end

			local num = ft[i] and tnum(ft[i]) or 0
			if num > 0 then
				wp.flags = num
			else
				wp.flags = 0
			end

			wp.id = ADGraphManager:getWayPointsCount() + i
			AutoDrive.dumpTable(wp, "wp", 2)
			table.insert(AutoDrivePlaceableData.wayPoints, wp) -- collect all waypoints to be created, but do it only at end if everthing is fine
			i = i + 1
		end
	end

	-- user confirmed import
	if AutoDrivePlaceableData.mapMarkers and #AutoDrivePlaceableData.mapMarkers > 0
		and AutoDrivePlaceableData.wayPoints and #AutoDrivePlaceableData.wayPoints > 0 then

		for _, mapMarker in pairs(AutoDrivePlaceableData.mapMarkers) do
			ADGraphManager:createMapMarker(mapMarker.id, mapMarker.name)
		end

		for _, wp in pairs(AutoDrivePlaceableData.wayPoints) do
			ADGraphManager:createWayPointWithConnections(wp.x, wp.y, wp.z, wp.out, wp.incoming, wp.flags)
		end
	end
	return 0 -- OK
end
