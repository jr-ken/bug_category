/*******************************************************************************
 * Copyright (c) 2017 Eurotech and/or its affiliates and others
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *     Eurotech - initial API and implementation
 *******************************************************************************/
package org.eclipse.kapua.app.console.module.authorization.server;

import java.util.ArrayList;
import java.util.List;

import org.eclipse.kapua.app.console.module.api.server.KapuaRemoteServiceServlet;
import org.eclipse.kapua.app.console.module.api.server.util.KapuaExceptionHandler;
import org.eclipse.kapua.app.console.module.api.client.GwtKapuaException;
import org.eclipse.kapua.app.console.module.api.shared.model.GwtXSRFToken;
import org.eclipse.kapua.app.console.module.api.shared.util.GwtKapuaCommonsModelConverter;
import org.eclipse.kapua.app.console.module.authorization.shared.model.GwtAccessRole;
import org.eclipse.kapua.app.console.module.authorization.shared.model.GwtAccessRoleCreator;
import org.eclipse.kapua.app.console.module.authorization.shared.model.GwtAccessRoleQuery;
import org.eclipse.kapua.app.console.module.authorization.shared.service.GwtAccessRoleService;
import org.eclipse.kapua.app.console.module.authorization.shared.util.GwtKapuaAuthorizationModelConverter;
import org.eclipse.kapua.app.console.module.authorization.shared.util.KapuaGwtAuthorizationModelConverter;
import org.eclipse.kapua.commons.model.query.FieldSortCriteria;
import org.eclipse.kapua.commons.model.query.FieldSortCriteria.SortOrder;
import org.eclipse.kapua.commons.model.query.predicate.AttributePredicate;
import org.eclipse.kapua.locator.KapuaLocator;
import org.eclipse.kapua.model.id.KapuaId;
import org.eclipse.kapua.service.authorization.access.AccessInfo;
import org.eclipse.kapua.service.authorization.access.AccessInfoService;
import org.eclipse.kapua.service.authorization.access.AccessPermissionListResult;
import org.eclipse.kapua.service.authorization.access.AccessRole;
import org.eclipse.kapua.service.authorization.access.AccessRoleCreator;
import org.eclipse.kapua.service.authorization.access.AccessRoleFactory;
import org.eclipse.kapua.service.authorization.access.AccessRoleListResult;
import org.eclipse.kapua.service.authorization.access.AccessRoleQuery;
import org.eclipse.kapua.service.authorization.access.AccessRoleService;
import org.eclipse.kapua.service.authorization.role.Role;
import org.eclipse.kapua.service.authorization.role.RoleService;
import org.eclipse.kapua.service.user.User;
import org.eclipse.kapua.service.user.UserService;

import com.extjs.gxt.ui.client.data.BasePagingLoadResult;
import com.extjs.gxt.ui.client.data.PagingLoadConfig;
import com.extjs.gxt.ui.client.data.PagingLoadResult;

public class GwtAccessRoleServiceImpl extends KapuaRemoteServiceServlet implements GwtAccessRoleService {

    private static final long serialVersionUID = 3606053200278262228L;

    @Override
    public GwtAccessRole create(GwtXSRFToken xsrfToken, GwtAccessRoleCreator gwtAccessRoleCreator) throws GwtKapuaException {

        //
        // Checking XSRF token
        checkXSRFToken(xsrfToken);

        //
        // Do create
        GwtAccessRole gwtAccessRole = null;
        try {
            // Convert from GWT Entity
            AccessRoleCreator accessRoleCreator = GwtKapuaAuthorizationModelConverter.convertAccessRoleCreator(gwtAccessRoleCreator);

            // Create
            KapuaLocator locator = KapuaLocator.getInstance();
            AccessRoleService accessRoleService = locator.getService(AccessRoleService.class);
            AccessRole accessRole = accessRoleService.create(accessRoleCreator);

            // Convert
            gwtAccessRole = KapuaGwtAuthorizationModelConverter.convertAccessRole(accessRole);

        } catch (Throwable t) {
            KapuaExceptionHandler.handle(t);
        }

        //
        // Return result
        return gwtAccessRole;
    }

    @Override
    public void delete(GwtXSRFToken gwtXsrfToken, String scopeShortId, String accessRoleShortId) throws GwtKapuaException {

        //
        // Checking XSRF token
        checkXSRFToken(gwtXsrfToken);

        //
        // Do delete
        try {
            // Convert from GWT Entity
            KapuaId scopeId = GwtKapuaCommonsModelConverter.convertKapuaId(scopeShortId);
            KapuaId accessRoleId = GwtKapuaCommonsModelConverter.convertKapuaId(accessRoleShortId);

            // Delete
            KapuaLocator locator = KapuaLocator.getInstance();
            AccessRoleService accessRoleService = locator.getService(AccessRoleService.class);
            accessRoleService.delete(scopeId, accessRoleId);
        } catch (Throwable t) {
            KapuaExceptionHandler.handle(t);
        }
    }

    @Override
    public PagingLoadResult<GwtAccessRole> findByUserId(PagingLoadConfig loadConfig, String scopeShortId, String userShortId) throws GwtKapuaException {
        //
        // Do get
        List<GwtAccessRole> gwtAccessRoles = new ArrayList<GwtAccessRole>();
        if (userShortId != null) {

            try {
                KapuaLocator locator = KapuaLocator.getInstance();
                RoleService roleService = locator.getService(RoleService.class);
                AccessInfoService accessInfoService = locator.getService(AccessInfoService.class);
                AccessRoleService accessRoleService = locator.getService(AccessRoleService.class);
                AccessRoleFactory accessRoleFactory = locator.getFactory(AccessRoleFactory.class);
                UserService userService = locator.getService(UserService.class);
                AccessRoleQuery accessRoleQuery = GwtKapuaAuthorizationModelConverter
                        .convertAccessRoleQuery(loadConfig, gwtRoleQuery);

                KapuaId scopeId = GwtKapuaCommonsModelConverter.convertKapuaId(scopeShortId);
                KapuaId userId = GwtKapuaCommonsModelConverter.convertKapuaId(userShortId);

                AccessInfo accessInfo = accessInfoService.findByUserId(scopeId, userId);
                User user = userService.find(scopeId, userId);

                if (accessInfo != null) {
                    AccessRoleQuery query = accessRoleFactory.newQuery(scopeId);
                    query.setPredicate(new AttributePredicate<KapuaId>("accessInfoId", accessInfo.getId()));
                    query.setLimit(loadConfig.getLimit());
                    query.setOffset(loadConfig.getOffset());
                    String sortField = StringUtils.isEmpty(loadConfig.getSortField()) ? "createdOn" : loadConfig.getSortField();
                    SortOrder sortOrder = loadConfig.getSortDir().equals(SortDir.DESC) ? SortOrder.DESCENDING : SortOrder.ASCENDING;
                    FieldSortCriteria sortCriteria = new FieldSortCriteria(sortField, sortOrder);
                    query.setSortCriteria(sortCriteria);
                    AccessRoleListResult accessRoleList = accessRoleService.query(query);
                    if (!accessRoleList.isEmpty()) {
                        totalLegnth = Long.valueOf(accessRoleService.count(query)).intValue();
                    }

                    for (AccessRole accessRole : accessRoleList.getItems()) {
                        Role role = roleService.find(scopeId, accessRole.getRoleId());
                        GwtAccessRole gwtAccessRole = KapuaGwtAuthorizationModelConverter.mergeRoleAccessRole(role, accessRole);
                        gwtAccessRole.setUserName(user.getName());
                        gwtAccessRoles.add(gwtAccessRole);
                    }
                }
            } catch (Throwable t) {
                KapuaExceptionHandler.handle(t);
            }
        }
        return new BasePagingLoadResult<GwtAccessRole>(gwtAccessRoles, loadConfig.getOffset(), totalLegnth);
    }
}
