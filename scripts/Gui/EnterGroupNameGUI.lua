--
-- AutoDrive Enter Group Name GUI
-- V1.1.0.0
--
-- @author Stephan Schlosser
-- @date 09/06/2019

ADEnterGroupNameGui = {}

local ADEnterGroupNameGui_mt = Class(ADEnterGroupNameGui, DialogElement)

function ADEnterGroupNameGui.new(target)
    local self = DialogElement.new(target, ADEnterGroupNameGui_mt)
    return self
end

function ADEnterGroupNameGui:onOpen()
    ADEnterGroupNameGui:superClass().onOpen(self)
    self.textInputElement.blockTime = 0
    self.textInputElement:onFocusActivate()
--     if self.textInputElement.overlay and self.textInputElement.overlay.colorFocused then
--         if AutoDrive.currentColors and AutoDrive.currentColors.ad_color_textInputBackground then
--             self.textInputElement.overlay.colorFocused = AutoDrive.currentColors.ad_color_textInputBackground
--         end
--     end
    self.textInputElement:setText("")
end

function ADEnterGroupNameGui:onClickOk()
    ADEnterGroupNameGui:superClass().onClickOk(self)

    if  self.textInputElement.text ~= ADGraphManager.debugGroupName then
        -- do not allow user to create debug group
        local groupName = self.textInputElement.text
        groupName = string.gsub(groupName, ",", "_") -- remove separation characters
        groupName = string.gsub(groupName, ";", "_")
        ADGraphManager:addGroup(groupName)
    end
    
    self:onClickBack()
end

function ADEnterGroupNameGui:onEnterPressed(_, isClick)
    if not isClick then
        self:onClickOk()
    end
end

function ADEnterGroupNameGui:onEscPressed()
    self:onClickBack()
end
