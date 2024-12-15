--
-- AutoDrive GUI
-- V1.0.0.0
--
-- @author TyKonKet
-- @date 03/12/2019

ADDebugSettingsPage = {}

local ADDebugSettingsPage_mt = Class(ADDebugSettingsPage, TabbedMenuFrameElement)

ADDebugSettingsPage.CONTROLS = {"settingsContainer", "headerIcon", "boxLayout"}

function ADDebugSettingsPage:new(target)
    local element = TabbedMenuFrameElement.new(target, ADDebugSettingsPage_mt)
    element.returnScreenName = ""
    element.debugElements = {}
    element.skipRow = 0

    element.lastDebugChannelMask = AutoDrive.currentDebugChannelMask
    return element
end

function ADDebugSettingsPage:onFrameOpen()
    ADDebugSettingsPage:superClass().onFrameOpen(self)
    self:updateDebugElements()
	FocusManager:setFocus(self.boxLayout)
end

function ADDebugSettingsPage:onFrameClose()
    ADDebugSettingsPage:superClass().onFrameClose(self)
end


function ADDebugSettingsPage:onCreateAutoDriveDebugSettingRow(element, channel)
    if self.skipRow == 0 then
        element:setImageColor(nil, 0.04231, 0.04231, 0.04231, 1)    
    else
        element:setImageColor(nil, 0.02956, 0.02956, 0.02956, 0.5)
    end
    self.skipRow = (self.skipRow + 1) % 2

    element.toggle = element.elements[1]
    element.toggle.debugChannel = tonumber(channel)    
    
    table.insert(self.debugElements, element)
end

function ADDebugSettingsPage:update(dt)
    ADDebugSettingsPage:superClass().update(self, dt)
    if self.lastDebugChannelMask ~= AutoDrive.currentDebugChannelMask then
        self:updateDebugElements()
        self.lastDebugChannelMask = AutoDrive.currentDebugChannelMask
    end
end

function ADDebugSettingsPage:onClickDebug(value, toggle)
    AutoDrive:setDebugChannel(toggle.debugChannel)
end

function ADDebugSettingsPage:updateDebugElements()
    for _, element in pairs(self.debugElements) do        
        local dbgChannel = element.toggle.debugChannel
        if dbgChannel ~= AutoDrive.DC_ALL then
            element.toggle:setIsChecked(AutoDrive.getDebugChannelIsSet(dbgChannel))
        else
            element.toggle:setIsChecked(AutoDrive.currentDebugChannelMask == AutoDrive.DC_ALL)
        end        
    end
end

function ADDebugSettingsPage:setupMenuButtonInfo(parent)
    local menuButtonInfo = {{inputAction = InputAction.MENU_BACK, text = g_i18n:getText("button_back"), callback = parent:makeSelfCallback(parent.onButtonBack), showWhenPaused = true}}
    self:setMenuButtonInfo(menuButtonInfo)
end

----- Get the frame's main content element's screen size.
function ADDebugSettingsPage:getMainElementSize()
    return self.settingsContainer.size
end

--- Get the frame's main content element's screen position.
function ADDebugSettingsPage:getMainElementPosition()
    return self.settingsContainer.absPosition
end

function ADDebugSettingsPage:copyAttributes(src)
	ADDebugSettingsPage:superClass().copyAttributes(self, src)
    self.storedHeaderKey = src.storedHeaderKey
    self.storedKey1 = src.storedKey1
end