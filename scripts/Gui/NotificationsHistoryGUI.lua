ADNotificationsHistoryGui = {}
ADNotificationsHistoryGui.debug = false

ADNotificationsHistoryGui.ICON_UVS = {
    { 0,   768, 256, 256 },
    { 256, 768, 256, 256 },
    { 512, 768, 256, 256 }
}

local ADNotificationsHistoryGui_mt = Class(ADNotificationsHistoryGui, DialogElement)

function ADNotificationsHistoryGui.new(target)
    local self = DialogElement.new(target, ADNotificationsHistoryGui_mt)
    self:include(ADGuiDebugMixin)
    self.history = {}
    return self
end

function ADNotificationsHistoryGui:onOpen()
    self:debugMsg("ADNotificationsHistoryGui:onOpen")
    ADNotificationsHistoryGui:superClass().onOpen(self)
    self:refreshItems()
    self.notificationsList:setDataSource(self)
end

function ADNotificationsHistoryGui:getNumberOfItemsInSection(list, section)
    self:debugMsg("ADNotificationsHistoryGui:getNumberOfItemsInSection")
    if list == self.notificationsList then
        return #self.history
    end
end

function ADNotificationsHistoryGui:populateCellForItemInSection(list, section, index, cell)
    self:debugMsg("ADNotificationsHistoryGui:populateCellForItemInSection")
    if list == self.notificationsList then
        local item = self.history[index]
        cell.attributes.listItemIcon:setImageUVs(nil, unpack(GuiUtils.getUVs(self.ICON_UVS[item.messageType])))
        cell.attributes.listItemText:setText(item.text)
        cell.target = self
    end
end

function ADNotificationsHistoryGui:refreshItems()
    self:debugMsg("ADNotificationsHistoryGui:refreshItems")
    self.history = ADMessagesManager:getHistory()
    self.notificationsList:reloadData()
end

function ADNotificationsHistoryGui:onDoubleClick(list, section, index, cell)
    self:debugMsg("ADNotificationsHistoryGui:onDoubleClick")
    if index > 0 and index <= #self.history then
        -- goto vehicle
        local v = self.history[index].vehicle
        if v ~= nil then
            self:onClickBack()
            AutoDrive.requestToEnterVehicle(v)
        end
    end
end

function ADNotificationsHistoryGui:onClickDeleteSelected()
    self:debugMsg("ADNotificationsHistoryGui:onClickDeleteSelected")
    if #self.history > 0 then
        local idx = self.notificationsList:getSelectedIndexInSection()
        ADMessagesManager:removeFromHistory(idx)
    end
end

function ADNotificationsHistoryGui:onClickDeleteAll()
    self:debugMsg("ADNotificationsHistoryGui:onClickDeleteAll")
    ADMessagesManager:clearHistory()
end

