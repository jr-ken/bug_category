/*******************************************************************************
 * Copyright (c) 2018 Eurotech and/or its affiliates and others
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *     Eurotech - initial API and implementation
 *******************************************************************************/
package org.eclipse.kapua.app.console.module.endpoint.client;

import org.eclipse.kapua.app.console.module.api.client.ui.dialog.KapuaDialog;
import org.eclipse.kapua.app.console.module.api.client.ui.widget.EntityCRUDToolbar;
import org.eclipse.kapua.app.console.module.api.shared.model.session.GwtSession;
import org.eclipse.kapua.app.console.module.endpoint.shared.model.GwtEndpoint;
import org.eclipse.kapua.app.console.module.endpoint.shared.model.permission.EndpointSessionPermission;
import com.extjs.gxt.ui.client.event.Events;

public class EndpointToolbarGrid extends EntityCRUDToolbar<GwtEndpoint> {

    public EndpointToolbarGrid(GwtSession currentSession) {
        super(currentSession);
    }

    public EndpointToolbarGrid(GwtSession currentSession, boolean slaveEntity) {
        super(currentSession, slaveEntity);
    }

    @Override
    protected KapuaDialog getAddDialog() {
        EndpointAddDialog dialog = new EndpointAddDialog(currentSession);
        dialog.addListener(Events.Hide, getHideDialogListener());
        return dialog;
    }

    @Override
    protected KapuaDialog getEditDialog() {
        GwtEndpoint selectedEndpoint = gridSelectionModel.getSelectedItem();
        EndpointEditDialog dialog = null;
        if (selectedEndpoint != null) {
            dialog = new EndpointEditDialog(currentSession, selectedEndpoint);
            dialog.addListener(Events.Hide, getHideDialogListener());
        }
        return dialog;
    }

    @Override
    protected KapuaDialog getDeleteDialog() {
        GwtEndpoint selectedEndpoint = gridSelectionModel.getSelectedItem();
        EndpointDeleteDialog dialog = null;
        if (selectedEndpoint != null) {
            dialog = new EndpointDeleteDialog(selectedEndpoint);
            dialog.addListener(Events.Hide, getHideDialogListener());
        }
        return dialog;
    }

    @Override
    protected void updateButtonEnablement() {
        super.updateButtonEnablement();

        //
        // Force disabled if entity is inherited from parent scopes
        addEntityButton.setEnabled(currentSession.hasPermission(EndpointSessionPermission.writeAll()));

        editEntityButton.setEnabled(selectedEntity != null && currentSession.hasPermission(EndpointSessionPermission.writeAll()));
        deleteEntityButton.setEnabled(selectedEntity != null && currentSession.hasPermission(EndpointSessionPermission.deleteAll()));
    }
}
