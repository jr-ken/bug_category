/*******************************************************************************
 * Copyright (c) 2019 Eurotech and/or its affiliates and others
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *     Eurotech - initial API and implementation
 *******************************************************************************/
package org.eclipse.kapua.app.console.module.device.shared.model.permission;

import org.eclipse.kapua.app.console.module.api.shared.model.session.GwtSessionPermission;
import org.eclipse.kapua.app.console.module.api.shared.model.session.GwtSessionPermissionAction;
import org.eclipse.kapua.app.console.module.api.shared.model.session.GwtSessionPermissionScope;

public class DeviceManagementRegistrySessionPermission extends GwtSessionPermission {

    private static final long serialVersionUID = 1L;

    public DeviceManagementRegistrySessionPermission() {
        super();
    }

    private DeviceManagementRegistrySessionPermission(GwtSessionPermissionAction action) {
        super("device_management_registry", action, GwtSessionPermissionScope.SELF);
    }

    public static DeviceManagementRegistrySessionPermission read() {
        return new DeviceManagementRegistrySessionPermission(GwtSessionPermissionAction.read);
    }

    public static DeviceManagementRegistrySessionPermission write() {
        return new DeviceManagementRegistrySessionPermission(GwtSessionPermissionAction.write);
    }

    public static DeviceManagementRegistrySessionPermission delete() {
        return new DeviceManagementRegistrySessionPermission(GwtSessionPermissionAction.delete);
    }
}
