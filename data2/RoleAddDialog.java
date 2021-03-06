/*******************************************************************************
 * Copyright (c) 2011, 2018 Eurotech and/or its affiliates and others
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *     Eurotech - initial API and implementation
 *******************************************************************************/
package org.eclipse.kapua.app.console.module.authorization.client.role.dialog;

import org.eclipse.kapua.app.console.module.api.client.ui.dialog.entity.EntityAddEditDialog;
import org.eclipse.kapua.app.console.module.api.client.ui.panel.FormPanel;
import org.eclipse.kapua.app.console.module.api.client.util.DialogUtils;
import org.eclipse.kapua.app.console.module.api.client.util.FailureHandler;
import org.eclipse.kapua.app.console.module.api.shared.model.GwtSession;
import org.eclipse.kapua.app.console.module.authorization.client.messages.ConsoleRoleMessages;
import org.eclipse.kapua.app.console.module.authorization.shared.model.GwtRole;
import org.eclipse.kapua.app.console.module.authorization.shared.model.GwtRoleCreator;
import org.eclipse.kapua.app.console.module.authorization.shared.service.GwtRoleService;

import com.extjs.gxt.ui.client.widget.form.TextField;
import com.google.gwt.core.client.GWT;
import com.google.gwt.user.client.rpc.AsyncCallback;
import org.eclipse.kapua.app.console.module.authorization.shared.service.GwtRoleServiceAsync;

public class RoleAddDialog extends EntityAddEditDialog {

    private final static ConsoleRoleMessages MSGS = GWT.create(ConsoleRoleMessages.class);

    private final static GwtRoleServiceAsync GWT_ROLE_SERVICE = GWT.create(GwtRoleService.class);

    protected TextField<String> roleNameField;

    public RoleAddDialog(GwtSession currentSession) {
        super(currentSession);

        DialogUtils.resizeDialog(this, 400, 150);
    }

    @Override
    public void submit() {
        GwtRoleCreator gwtRoleCreator = new GwtRoleCreator();

        gwtRoleCreator.setScopeId(currentSession.getSelectedAccountId());
        gwtRoleCreator.setName(roleNameField.getValue());

        GWT_ROLE_SERVICE.create(xsrfToken, gwtRoleCreator, new AsyncCallback<GwtRole>() {

            @Override
            public void onSuccess(GwtRole arg0) {
                exitStatus = true;
                exitMessage = MSGS.dialogAddConfirmation();
                hide();
            }

            @Override
            public void onFailure(Throwable cause) {
                FailureHandler.handleFormException(formPanel, cause);
                status.hide();
                formPanel.getButtonBar().enable();
                unmask();
                submitButton.enable();
                cancelButton.enable();
                hide();
            }
        });

    }

    @Override
    public String getHeaderMessage() {
        return MSGS.dialogAddHeader();
    }

    @Override
    public String getInfoMessage() {
        return MSGS.dialogAddInfo();
    }

    @Override
    public void createBody() {
        FormPanel roleFormPanel = new FormPanel(FORM_LABEL_WIDTH);

        //
        // Name
        roleNameField = new TextField<String>();
        roleNameField.setAllowBlank(false);
        roleNameField.setFieldLabel("* " + MSGS.dialogAddFieldName());
        roleNameField.setToolTip(MSGS.dialogAddFieldNameTooltip());
        roleFormPanel.add(roleNameField);

        bodyPanel.add(roleFormPanel);
    }

}
