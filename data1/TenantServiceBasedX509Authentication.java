/**
 * Copyright (c) 2019 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 */


package org.eclipse.hono.service.auth.device;

import java.net.HttpURLConnection;
import java.security.cert.Certificate;
import java.security.cert.CertificateEncodingException;
import java.security.cert.TrustAnchor;
import java.security.cert.X509Certificate;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import javax.security.auth.x500.X500Principal;

import org.eclipse.hono.client.ClientErrorException;
import org.eclipse.hono.client.ServiceInvocationException;
import org.eclipse.hono.client.TenantClientFactory;
import org.eclipse.hono.tracing.TracingHelper;
import org.eclipse.hono.util.CredentialsConstants;
import org.eclipse.hono.util.RequestResponseApiConstants;
import org.eclipse.hono.util.TenantObject;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.opentracing.Span;
import io.opentracing.SpanContext;
import io.opentracing.Tracer;
import io.opentracing.noop.NoopTracerFactory;
import io.opentracing.tag.Tags;
import io.vertx.core.Future;
import io.vertx.core.json.JsonObject;

/**
 * A service for validating X.509 client certificates against
 * a trust anchor maintained in a Hono Tenant service.
 * <p>
 * The trust anchor is determined by looking up the tenant that the device
 * belongs to using the client certificate's issuer DN as described by the
 * <a href="https://www.eclipse.org/hono/docs/api/tenant-api/#get-tenant-information">
 * Tenant API</a>.
 *
 */
public final class TenantServiceBasedX509Authentication implements X509Authentication {

    private static final ClientErrorException UNAUTHORIZED = new ClientErrorException(HttpURLConnection.HTTP_UNAUTHORIZED);
    private static final Logger log = LoggerFactory.getLogger(TenantServiceBasedX509Authentication.class);

    private final Tracer tracer;
    private final TenantClientFactory tenantClientFactory;
    private final DeviceCertificateValidator certPathValidator;

    /**
     * Creates a new instance for a Tenant service client.
     * 
     * @param tenantClientFactory The factory to use for creating a Tenant service client.
     */
    public TenantServiceBasedX509Authentication(final TenantClientFactory tenantClientFactory) {

        this(tenantClientFactory, NoopTracerFactory.create());
    }

    /**
     * Creates a new instance for a Tenant service client.
     * 
     * @param tenantClientFactory The factory to use for creating a Tenant service client.
     * @param tracer The <em>OpenTracing</em> tracer to use for tracking the process of
     *               authenticating the client.
     */
    public TenantServiceBasedX509Authentication(
            final TenantClientFactory tenantClientFactory,
            final Tracer tracer) {
        this(tenantClientFactory, tracer, new DeviceCertificateValidator());
    }

    /**
     * Creates a new instance for a Tenant service client.
     * 
     * @param tenantClientFactory The factory to use for creating a Tenant service client.
     * @param tracer The <em>OpenTracing</em> tracer to use for tracking the process of
     *               authenticating the client.
     * @param certPathValidator The validator to use for establishing the client certificate's
     *                          chain of trust.
     */
    public TenantServiceBasedX509Authentication(
            final TenantClientFactory tenantClientFactory,
            final Tracer tracer,
            final DeviceCertificateValidator certPathValidator) {

        this.tenantClientFactory = Objects.requireNonNull(tenantClientFactory);
        this.tracer = Objects.requireNonNull(tracer);
        this.certPathValidator = Objects.requireNonNull(certPathValidator);
    }

    /**
     * Validates a certificate path using a trust anchor retrieved from
     * the Tenant service.
     * 
     * @param path The certificate path to validate.
     * @param currentSpan The <em>OpenTracing</em> context in which the
     *                    validation should be executed, or {@code null}
     *                    if no context exists (yet).
     * @return A future indicating the outcome of the validation.
     *         <p>
     *         The future will be failed with a {@link ServiceInvocationException}
     *         if the certificate path could not be validated.
     *         <p>
     *         Otherwise, the future will be succeeded with a JSON object having
     *         the following properties:
     *         <pre>
     *         {
     *           "subject-dn": [RFC 2253 formatted subject DN of the client's end certificate],
     *           "tenant-id": [identifier of the tenant that the device belongs to]
     *         }
     *         </pre>
     *
     *         If auto-provisioning is enabled for the trust anchor being used, the JSON object may optionally contain
     *         the DER encoding of the (validated) client certificate as a Base64 encoded byte array in the
     *         client-certificate property.
     * @throws NullPointerException if certificate path is {@code null}.
     */
    @Override
    public Future<JsonObject> validateClientCertificate(
            final Certificate[] path,
            final SpanContext currentSpan) {

        Objects.requireNonNull(path);

        final Span span = TracingHelper.buildChildSpan(tracer, currentSpan, "verify device certificate")
                .ignoreActiveSpan()
                .withTag(Tags.SPAN_KIND.getKey(), Tags.SPAN_KIND_CLIENT)
                .withTag(Tags.COMPONENT.getKey(), getClass().getSimpleName())
                .start();

        return getX509CertificatePath(path).compose(x509chain -> {

            final X509Certificate deviceCert = x509chain.get(0);
            final Map<String, String> detail = new HashMap<>(3);
            detail.put("subject DN", deviceCert.getSubjectX500Principal().getName());
            detail.put("not before", deviceCert.getNotBefore().toString());
            detail.put("not after", deviceCert.getNotAfter().toString());
            span.log(detail);

            final Future<TenantObject> tenantTracker = getTenant(deviceCert, span);
            return tenantTracker
                    .compose(tenant -> {
                        final Set<TrustAnchor> trustAnchors = tenant.getTrustAnchors();
                        if (trustAnchors.isEmpty()) {
                            log.debug("no valid trust anchors defined for tenant [{}]", tenant.getTenantId());
                            return Future.failedFuture(UNAUTHORIZED);
                        } else {
                            final List<X509Certificate> chainToValidate = Collections.singletonList(deviceCert);
                            return certPathValidator.validate(chainToValidate, trustAnchors)
                                    .recover(t -> Future.failedFuture(UNAUTHORIZED));
                        }
                    }).compose(ok -> getCredentials(x509chain, tenantTracker.result()));
        }).map(authInfo -> {
            span.log("certificate verified successfully");
            span.finish();
            return authInfo;
        }).recover(t -> {
            log.debug("verification of client certificate failed: {}", t.getMessage());
            TracingHelper.logError(span, t);
            span.finish();
            return Future.failedFuture(t);
        });
    }

    private Future<TenantObject> getTenant(final X509Certificate clientCert, final Span span) {

        return tenantClientFactory.getOrCreateTenantClient().compose(tenantClient ->
            tenantClient.get(clientCert.getIssuerX500Principal(), span.context()));
    }

    private Future<List<X509Certificate>> getX509CertificatePath(final Certificate[] clientPath) {

        final List<X509Certificate> path = new LinkedList<>();
        for (Certificate cert : clientPath) {
            if (cert instanceof X509Certificate) {
                path.add((X509Certificate) cert);
            } else {
                log.info("cannot authenticate device using unsupported certificate type [{}]",
                        cert.getClass().getName());
                return Future.failedFuture(UNAUTHORIZED);
            }
        }
        return Future.succeededFuture(path);
    }

    /**
     * Gets the authentication information for a device's client certificate.
     * <p>
     * This returns a JSON object that contains the following properties:
     * <ul>
     * <li>{@link RequestResponseApiConstants#FIELD_PAYLOAD_SUBJECT_DN} -
     * the subject DN from the certificate (<em>mandatory</em>)</li>
     * <li>{@link RequestResponseApiConstants#FIELD_PAYLOAD_TENANT_ID} -
     * the identifier of the tenant that the device belongs to (<em>mandatory</em>)</li>
     * <li>{@link CredentialsConstants#FIELD_CLIENT_CERT} -
     * the client certificate that the device used for authenticating as Base64  encoded
     * byte array as returned by {@link java.security.cert.X509Certificate#getEncoded()}
     * (<em>optional: only present if auto-provisioning is enabled for the used trust anchor</em>)</li>
     * </ul>
     * 
     * @param clientCertPath The validated client certificate path that the device has
     *                   presented during the TLS handshake. The device's end certificate
     *                   is contained at index 0.
     * @param tenant The tenant that the device belongs to.
     * @return A succeeded future containing the authentication information that will be passed on
     *         to the {@code AuthProvider} for verification. The future will be
     *         failed if the information cannot be extracted from the certificate chain.
     */
    protected Future<JsonObject> getCredentials(final List<X509Certificate> clientCertPath, final TenantObject tenant) {

        final X509Certificate deviceCert = clientCertPath.get(0);
        final String subjectDn = deviceCert.getSubjectX500Principal().getName(X500Principal.RFC2253);
        log.debug("authenticating device of tenant [{}] using X509 certificate [subject DN: {}]",
                tenant.getTenantId(), subjectDn);

        final JsonObject authInfo = new JsonObject()
                .put(CredentialsConstants.FIELD_PAYLOAD_SUBJECT_DN, subjectDn)
                .put(CredentialsConstants.FIELD_PAYLOAD_TENANT_ID, tenant.getTenantId());

        final String issuerDn = deviceCert.getIssuerX500Principal().getName(X500Principal.RFC2253);
        if (tenant.isAutoProvisioningEnabled(issuerDn)) {
            try {
                authInfo.put(CredentialsConstants.FIELD_CLIENT_CERT, deviceCert.getEncoded());
            } catch (CertificateEncodingException e) {
                log.error("Encoding of device certificate failed [subject DN: {}]", subjectDn, e);
                return Future.failedFuture(e);
            }
        }
        return Future.succeededFuture(authInfo);
    }

}
