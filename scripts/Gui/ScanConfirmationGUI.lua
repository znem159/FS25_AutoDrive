--
-- AutoDrive Enter Target Name GUI
-- V1.1.0.0
--
-- @author Stephan Schlosser
-- @date 08/08/2019

ADScanConfirmationGui = {}

local ADScanConfirmationGui_mt = Class(ADScanConfirmationGui, DialogElement)

function ADScanConfirmationGui.new(target)
    local self = DialogElement.new(target, ADScanConfirmationGui_mt)
    return self
end

function ADScanConfirmationGui:onYes()
    AutoDrive.scanDialogState = AutoDrive.SCAN_DIALOG_RESULT_YES
    self:onClickBack()
end

function ADScanConfirmationGui:onNo()
    AutoDrive.scanDialogState = AutoDrive.SCAN_DIALOG_RESULT_NO
    self:onClickBack()
end


