--
-- AutoDrive Enter filter for destinations shown in drop down menus GUI
-- V1.1.0.0
--
-- @author Stephan Schlosser
-- @date 09/06/2019

ADEnterDestinationFilterGui = {}

local ADEnterDestinationFilterGui_mt = Class(ADEnterDestinationFilterGui, DialogElement)

function ADEnterDestinationFilterGui.new(target)
    local self = DialogElement.new(target, ADEnterDestinationFilterGui_mt)
    return self
end

function ADEnterDestinationFilterGui:onOpen()
    ADEnterDestinationFilterGui:superClass().onOpen(self)
    local controlledVehicle = AutoDrive.getControlledVehicle()
    self.textInputElement.blockTime = 0
    self.textInputElement:onFocusActivate()
--     if self.textInputElement.overlay and self.textInputElement.overlay.colorFocused then
--         if AutoDrive.currentColors and AutoDrive.currentColors.ad_color_textInputBackground then
--             self.textInputElement.overlay.colorFocused = AutoDrive.currentColors.ad_color_textInputBackground
--         end
--     end
    if controlledVehicle ~= nil and controlledVehicle.ad ~= nil then
        self.textInputElement:setText(controlledVehicle.ad.destinationFilterText)
    end
end

function ADEnterDestinationFilterGui:onClickOk()
    ADEnterDestinationFilterGui:superClass().onClickOk(self)
    local controlledVehicle = AutoDrive.getControlledVehicle()
    if controlledVehicle ~= nil and controlledVehicle.ad ~= nil then
        controlledVehicle.ad.destinationFilterText = self.textInputElement.text
    end
    self:onClickBack()
end

function ADEnterDestinationFilterGui:onClickCancel()
    self.textInputElement:setText("")
end

function ADEnterDestinationFilterGui:onEnterPressed(_, isClick)
    if not isClick then
        self:onClickOk()
    end
end

function ADEnterDestinationFilterGui:onEscPressed()
    self:onClickBack()
end
