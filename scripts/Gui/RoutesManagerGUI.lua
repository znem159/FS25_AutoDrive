ADRoutesManagerGui = {}

local ADRoutesManagerGui_mt = Class(ADRoutesManagerGui, DialogElement)

function ADRoutesManagerGui.new(target)
    local self = DialogElement.new(target, ADRoutesManagerGui_mt)
    self.routes = {}
    return self
end

function ADRoutesManagerGui:onOpen()
    --     if self.textInputElement.overlay and self.textInputElement.overlay.colorFocused then
    --         if AutoDrive.currentColors and AutoDrive.currentColors.ad_color_textInputBackground then
    --             self.textInputElement.overlay.colorFocused = AutoDrive.currentColors.ad_color_textInputBackground
    --         end
    --     end
    ADRoutesManagerGui:superClass().onOpen(self)
    self:refreshItems()
    self.routeList:setDataSource(self)
end

function ADRoutesManagerGui:getNumberOfItemsInSection(list, section)
    if list == self.routeList then
        return #self.routes
    end
end

function ADRoutesManagerGui:populateCellForItemInSection(list, section, index, cell)
    if list == self.routeList then
        cell.attributes.listItemText:setText(self.routes[index].name)
        cell.attributes.listItemDate:setText(self.routes[index].date)
        cell.target = self
    end
end

function ADRoutesManagerGui:refreshItems()
    self.routes = ADRoutesManager:getRoutes(AutoDrive.loadedMap)
    self.routeList:reloadData()
end

function ADRoutesManagerGui:onDoubleClick(list, section, index, cell)
    self.textInputElement:setText(self.routes[index].name)
end

function ADRoutesManagerGui:onClickOk()
    -- Save route
    ADRoutesManagerGui:superClass().onClickOk(self)
    local newName = self.textInputElement.text
    if table.f_contains(
            self.routes,
            function(v)
                return v.name == newName
            end
        )
    then
        AutoDrive.showYesNoDialog(
            g_i18n:getText("gui_ad_routeExportWarn_title"),
            g_i18n:getText("gui_ad_routeExportWarn_text"),
            self.onExportDialogCallback,
            self
        )
    else
        self:onExportDialogCallback(true)
    end
end

function ADRoutesManagerGui:onExportDialogCallback(yes)
    if yes then
        ADRoutesManager:export(self.textInputElement.text)
        self:refreshItems()
    end
end

function ADRoutesManagerGui:onClickCancel()
    -- Load route
    if #self.routes > 0 then
        ADRoutesManager:import(self.routes[self.routeList:getSelectedIndexInSection()].name)
        self:onClickBack()
    end
    ADRoutesManagerGui:superClass().onClickCancel(self)
end

function ADRoutesManagerGui:onClickActivate()
    -- Delete route
    if #self.routes > 0 then
        local idx = self.routeList:getSelectedIndexInSection()
        AutoDrive.showYesNoDialog(
            g_i18n:getText("gui_ad_routeDeleteWarn_title"),
            g_i18n:getText("gui_ad_routeDeleteWarn_text"):format(self.routes[idx].name),
            self.onDeleteDialogCallback,
            self,
            idx
        )
    end
    ADRoutesManagerGui:superClass().onClickActivate(self)
end

function ADRoutesManagerGui:onDeleteDialogCallback(yes, idx)
    if yes then
        ADRoutesManager:remove(self.routes[idx].name)
        self:refreshItems()
    end
end

function ADRoutesManagerGui:onEscPressed()
    -- ESC pressed in textInputElement, close dialog
    self:onClickBack()
end
