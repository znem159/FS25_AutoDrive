--
-- AutoDrive Enter Driver Name GUI
-- V1.1.0.0
--
-- @author Stephan Schlosser
-- @date 09/06/2019

ADEnterDriverNameGui = {}

local ADEnterDriverNameGui_mt = Class(ADEnterDriverNameGui, DialogElement)

function ADEnterDriverNameGui.new(target)
    local self = DialogElement.new(target, ADEnterDriverNameGui_mt)
    return self
end

function ADEnterDriverNameGui:onOpen()
    ADEnterDriverNameGui:superClass().onOpen(self)
    local controlledVehicle = AutoDrive.getControlledVehicle()
    self.textInputElement.blockTime = 0
    self.textInputElement:onFocusActivate()
--     if self.textInputElement.overlay and self.textInputElement.overlay.colorFocused then
--         if AutoDrive.currentColors and AutoDrive.currentColors.ad_color_textInputBackground then
--             self.textInputElement.overlay.colorFocused = AutoDrive.currentColors.ad_color_textInputBackground
--         end
--     end
    if controlledVehicle ~= nil and controlledVehicle.ad ~= nil then
        self.textInputElement:setText(controlledVehicle.ad.stateModule:getName())
    end
end

function ADEnterDriverNameGui:onClickRename()
    local controlledVehicle = AutoDrive.getControlledVehicle()
    if controlledVehicle ~= nil then
        AutoDrive.renameDriver(controlledVehicle, self.textInputElement.text)
    end
    self:onClickBack()
end

function ADEnterDriverNameGui:onClickReset()
    local controlledVehicle = AutoDrive.getControlledVehicle()
    if controlledVehicle ~= nil and controlledVehicle.ad ~= nil then
        self.textInputElement:setText(controlledVehicle.ad.stateModule:getName())
    end
end

function ADEnterDriverNameGui:onEnterPressed(_, isClick)
    if not isClick then
        self:onClickRename()
    end
end

function ADEnterDriverNameGui:onEscPressed()
    self:onClickBack()
end
