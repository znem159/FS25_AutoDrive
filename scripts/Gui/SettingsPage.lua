--
-- AutoDrive GUI
-- V1.0.0.0
--
-- @author Stephan Schlosser
-- @date 08/04/2019

ADSettingsPage = {}

local ADSettingsPage_mt = Class(ADSettingsPage, TabbedMenuFrameElement)

-- ADSettingsPage.CONTROLS = {"settingsContainer", "ingameMenuHelpBox", "headerIcon", "headerText"}
ADSettingsPage.CONTROLS = {"settingsContainer", "ingameMenuHelpBox", "boxLayout"}

function ADSettingsPage:new(target)
    local element = TabbedMenuFrameElement.new(target, ADSettingsPage_mt)
    element.returnScreenName = ""
    element.settingElements = {}
    element.skipRow = 0

    return element
end

function ADSettingsPage:onFrameOpen()
    ADSettingsPage:superClass().onFrameOpen(self)
    -- FocusManager:unsetHighlight(FocusManager.currentFocusData.highlightElement)
    -- FocusManager:unsetFocus(FocusManager.currentFocusData.focusElement)
    if not self:hasChanges() then
        self:loadGUISettings()
    end
    FocusManager:setFocus(self.boxLayout)    
end


function ADSettingsPage:onFrameClose()
    ADSettingsPage:superClass().onFrameClose(self)
end

function ADSettingsPage:onCreateAutoDriveSettingRow(element)
    if self.skipRow == 0 then
        element:setImageColor(nil, 0.04231, 0.04231, 0.04231, 1)    
    else
        element:setImageColor(nil, 0.02956, 0.02956, 0.02956, 0.5)
    end
    self.skipRow = (self.skipRow + 1) % 2
end

function ADSettingsPage:onCreateAutoDriveSetting(element)
    self.settingElements[element.name] = element

    local setting = AutoDrive.settings[element.name]

    local labels = {}
    for i = 1, #setting.texts, 1 do
        if setting.translate == true then
            local text = g_i18n:getText(setting.texts[i])
            local missingText = "Missing"
            if text:sub(1, string.len(missingText)) == missingText then
                labels[i] = setting.texts[i]
            else
                labels[i] = text
            end
        else
            labels[i] = setting.texts[i]
        end
    end    

    element:setTexts(labels)

    element.parent:setImageColor(nil, unpack(ADSettings.ICON_COLOR.DEFAULT))

    --[[
    local iconElem = element.elements[6]
    if iconElem ~= nil then
        if setting.isUserSpecific then
            iconElem:setImageFilename(g_autoDriveIconFilename)
            iconElem:setImageUVs(nil, unpack(GuiUtils.getUVs(ADSettings.ICON_UV.USER)))
        elseif setting.isVehicleSpecific then
            iconElem:setImageFilename(g_autoDriveIconFilename)
            iconElem:setImageUVs(nil, unpack(GuiUtils.getUVs(ADSettings.ICON_UV.VEHICLE)))
        else
            iconElem:setImageFilename(g_autoDriveIconFilename)
            iconElem:setImageUVs(nil, unpack(GuiUtils.getUVs(ADSettings.ICON_UV.GLOBAL)))
        end
    end
    --]]
end

function ADSettingsPage:onOptionChange(state, element)
    local setting = AutoDrive.settings[element.name]
    local controlledVehicle = AutoDrive.getControlledVehicle()
    if setting.isVehicleSpecific and controlledVehicle ~= nil and controlledVehicle.ad ~= nil and controlledVehicle.ad.settings[element.name] ~= nil then
        setting = controlledVehicle.ad.settings[element.name]
    end
    setting.new = state

    --[[
    local iconElem = element.elements[6]
    if iconElem ~= nil then
        if setting.new ~= setting.current then
            iconElem:setImageColor(iconElem.overlayState, unpack(ADSettings.ICON_COLOR.CHANGED))
        else
            iconElem:setImageColor(iconElem.overlayState, unpack(ADSettings.ICON_COLOR.DEFAULT))
        end
    end
    --]]
end

function ADSettingsPage:hasChanges()
    local controlledVehicle = AutoDrive.getControlledVehicle()
    for settingName, _ in pairs(self.settingElements) do
        if AutoDrive.settings[settingName] ~= nil then
            local setting = AutoDrive.settings[settingName]
            if setting.isVehicleSpecific and controlledVehicle ~= nil and controlledVehicle.ad ~= nil and controlledVehicle.ad.settings[settingName] ~= nil then
                setting = controlledVehicle.ad.settings[settingName]
            end
            if setting.new ~= nil and setting.new ~= setting.current then
                return true
            end
        end
    end
    
    return false
end

----- Get the frame's main content element's screen size.
function ADSettingsPage:getMainElementSize()
    return self.settingsContainer.size
end

--- Get the frame's main content element's screen position.
function ADSettingsPage:getMainElementPosition()
    return self.settingsContainer.absPosition
end

function ADSettingsPage:onIngameMenuHelpTextChanged(box)
    local hasText = box.text ~= nil and box.text ~= ""
    self.ingameMenuHelpBox:setVisible(hasText)
end

function ADSettingsPage:loadGUISettings()
    local controlledVehicle = AutoDrive.getControlledVehicle()
    for settingName, _ in pairs(self.settingElements) do
        if AutoDrive.settings[settingName] ~= nil then
            local setting = AutoDrive.settings[settingName]
            if setting.isVehicleSpecific and controlledVehicle ~= nil and controlledVehicle.ad ~= nil and controlledVehicle.ad.settings[settingName] ~= nil then
                setting = controlledVehicle.ad.settings[settingName]
            end
            self:loadGUISetting(settingName, setting.current)
        end
    end
end

function ADSettingsPage:loadGUISetting(settingName, state)
    local element = self.settingElements[settingName]
    element:setState(state, false)
    self:onOptionChange(state, element)
end

function ADSettingsPage:copyAttributes(src)
	ADSettingsPage:superClass().copyAttributes(self, src)
    self.storedHeaderKey = src.storedHeaderKey
    self.storedKey1 = src.storedKey1
end
