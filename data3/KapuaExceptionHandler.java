/*******************************************************************************
 * Copyright (c) 2017, 2018 Eurotech and/or its affiliates and others
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *     Eurotech - initial API and implementation
 *******************************************************************************/
package org.eclipse.kapua.app.console.module.api.server.util;

import org.eclipse.kapua.KapuaErrorCodes;
import org.eclipse.kapua.KapuaException;
import org.eclipse.kapua.KapuaRuntimeException;
import org.eclipse.kapua.KapuaUnauthenticatedException;
import org.eclipse.kapua.app.console.module.api.client.GwtKapuaErrorCode;
import org.eclipse.kapua.app.console.module.api.client.GwtKapuaException;
import org.eclipse.kapua.commons.configuration.KapuaConfigurationErrorCodes;
import org.eclipse.kapua.commons.configuration.KapuaConfigurationException;
import org.eclipse.kapua.service.authentication.shiro.KapuaAuthenticationErrorCodes;
import org.eclipse.kapua.service.authentication.shiro.KapuaAuthenticationException;
import org.eclipse.kapua.service.authorization.shiro.exception.SubjectUnauthorizedException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class KapuaExceptionHandler {

    private static final Logger logger = LoggerFactory.getLogger(KapuaExceptionHandler.class);

    private KapuaExceptionHandler() {
    }

    public static void handle(Throwable t) throws GwtKapuaException {
        if (t instanceof KapuaUnauthenticatedException) {

            // sessions has expired
            throw new GwtKapuaException(GwtKapuaErrorCode.UNAUTHENTICATED, t);
        } else if (t instanceof KapuaAuthenticationException) {

            final KapuaAuthenticationException ke = (KapuaAuthenticationException) t;
            final String cause = ke.getCode().name();

            // INVALID_USERNAME_PASSWORD

            if (cause.equals(KapuaAuthenticationErrorCodes.INVALID_LOGIN_CREDENTIALS.name())) {
                throw new GwtKapuaException(GwtKapuaErrorCode.INVALID_USERNAME_PASSWORD, t);
            }
            if (cause.equals(KapuaAuthenticationErrorCodes.UNKNOWN_LOGIN_CREDENTIAL.name())) {
                throw new GwtKapuaException(GwtKapuaErrorCode.INVALID_USERNAME_PASSWORD, t);
            }

            // LOCKED_USER

            if (cause.equals(KapuaAuthenticationErrorCodes.LOCKED_LOGIN_CREDENTIAL.name())) {
                throw new GwtKapuaException(GwtKapuaErrorCode.LOCKED_USER, t);
            }
            if (cause.equals(KapuaAuthenticationErrorCodes.DISABLED_LOGIN_CREDENTIAL.name())) {
                throw new GwtKapuaException(GwtKapuaErrorCode.LOCKED_USER, t);
            }
            if (cause.equals(KapuaAuthenticationErrorCodes.EXPIRED_LOGIN_CREDENTIALS.name())) {
                throw new GwtKapuaException(GwtKapuaErrorCode.LOCKED_USER, t);
            }

            // default
            throw new GwtKapuaException(GwtKapuaErrorCode.INVALID_USERNAME_PASSWORD, t);
        }
        else if (t instanceof KapuaRuntimeException && ((KapuaRuntimeException) t).getCode().equals(KapuaErrorCodes.ENTITY_ALREADY_EXISTS)) {
            logger.error("entity already exists", t);
            throw new GwtKapuaException(GwtKapuaErrorCode.ENTITY_ALREADY_EXISTS, t, t.getLocalizedMessage());
        } else if (t.getCause() instanceof KapuaRuntimeException && ((KapuaRuntimeException) t.getCause()).getCode().equals(KapuaErrorCodes.ENTITY_ALREADY_EXISTS)) {
            logger.error("entity already exists", t);
            throw new GwtKapuaException(GwtKapuaErrorCode.ENTITY_ALREADY_EXISTS, t, t.getLocalizedMessage());
        } else if (t instanceof KapuaException && ((KapuaException) t).getCode().equals(KapuaErrorCodes.INTERNAL_ERROR)) {
            logger.error("internal service error", t);
            throw new GwtKapuaException(GwtKapuaErrorCode.INTERNAL_ERROR, t, t.getLocalizedMessage());
        } else if(t instanceof KapuaException && ((KapuaException) t).getCode().name().equals(KapuaErrorCodes.PARENT_LIMIT_EXCEEDED_IN_CONFIG.name())) {
            logger.warn("Child accounts limitation error", t);
            throw new GwtKapuaException(GwtKapuaErrorCode.PARENT_LIMIT_EXCEEDED_IN_CONFIG, t, t.getLocalizedMessage());
        } else if(t instanceof KapuaException && ((KapuaException) t).getCode().name().equals(KapuaErrorCodes.SUBJECT_UNAUTHORIZED.name())) {
            logger.warn("User unauthorize", t);
            throw new GwtKapuaException(GwtKapuaErrorCode.SUBJECT_UNAUTHORIZED, t, ((SubjectUnauthorizedException)t).getPermission().toString());
        } else if (t instanceof KapuaException && ((KapuaException) t).getCode().name().equals(KapuaErrorCodes.ENTITY_ALREADY_EXIST_IN_ANOTHER_ACCOUNT.name())) {
            logger.warn("Entity already exist in another account", t);
            throw new GwtKapuaException(GwtKapuaErrorCode.ENTITY_ALREADY_EXIST_IN_ANOTHER_ACCOUNT, t, t.getLocalizedMessage());
        } else if (t instanceof KapuaException && ((KapuaException) t).getCode().name().equals(KapuaErrorCodes.DUPLICATE_NAME.name())) {
            logger.warn("Entity already exist with the same name", t);
            throw new GwtKapuaException(GwtKapuaErrorCode.DUPLICATE_NAME, t, t.getLocalizedMessage());
        } else if(t instanceof KapuaConfigurationException && ((KapuaConfigurationException) t).getCode().name().equals(KapuaConfigurationErrorCodes.SELF_LIMIT_EXCEEDED_IN_CONFIG.name())) {
            logger.warn("Parent account limitation error", t);
            throw new GwtKapuaException(GwtKapuaErrorCode.SELF_LIMIT_EXCEEDED_IN_CONFIG, t, t.getLocalizedMessage());
        } else {
            // all others => log and throw internal error code
            logger.warn("RPC service non-application error", t);
            throw GwtKapuaException.internalError(t, t.getLocalizedMessage());
        }
    }
}
