/**
 * Copyright (c) 2010-2019 Contributors to the openHAB project
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 */
package org.eclipse.smarthome.core.ephemeris.internal;

import static org.eclipse.smarthome.core.ephemeris.internal.EphemerisManagerImpl.*;
import static org.junit.Assert.*;

import java.net.URI;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.eclipse.jdt.annotation.NonNullByDefault;
import org.eclipse.smarthome.config.core.ParameterOption;
import org.eclipse.smarthome.core.ephemeris.EphemerisManager;
import org.eclipse.smarthome.test.java.JavaOSGiTest;
import org.junit.Before;
import org.junit.Test;

/**
 * Test class for {@link EphemerisManagerImpl}.
 *
 * @author Christoph Weitkamp - Initial contribution
 */
@NonNullByDefault
public class EphemerisManagerImplOSGiTest extends JavaOSGiTest {

    private static final String COUNTRY_AUSTRALIA_KEY = "au";
    private static final String COUNTRY_AUSTRALIA_NAME = "Australia";
    private static final String REGION_TASMANIA_KEY = "tas";
    private static final String REGION_TASMANIA_NAME = "Tasmania";
    private static final String CITY_HOBARD_AREA_KEY = "ho";
    private static final String CITY_HOBARD_AREA_NAME = "Hobard Area";

    private @NonNullByDefault({}) EphemerisManagerImpl ephemerisManager;

    @Before
    public void setUp() {
        ephemerisManager = getService(EphemerisManager.class, EphemerisManagerImpl.class);
        assertNotNull(ephemerisManager);

        ephemerisManager.modified(Collections.singletonMap(CONFIG_COUNTRY, Locale.GERMANY.getCountry()));
    }

    @Test
    public void testEphemerisManagerLoadedProperly() {
        assertFalse(ephemerisManager.countries.isEmpty());
        assertFalse(ephemerisManager.regions.isEmpty());
        assertFalse(ephemerisManager.cities.isEmpty());
    }

    @Test(expected = IllegalArgumentException.class)
    public void testParsePropertyFailed() {
        ephemerisManager.parseProperty("", "");
    }

    @Test
    public void testParsePropertyCountryCorrectly() {
        final Optional<ParameterOption> option = ephemerisManager.countries.stream()
                .filter(o -> COUNTRY_AUSTRALIA_KEY.equals(o.getValue())).findFirst();
        assertTrue(option.isPresent());
        assertEquals(COUNTRY_AUSTRALIA_KEY, option.get().getValue());
        assertEquals(COUNTRY_AUSTRALIA_NAME, option.get().getLabel());
    }

    @Test
    public void testParsePropertyRegionCorrectly() {
        assertTrue(ephemerisManager.regions.containsKey(COUNTRY_AUSTRALIA_KEY));
        final List<ParameterOption> regions = ephemerisManager.regions.get(COUNTRY_AUSTRALIA_KEY);
        assertFalse(regions.isEmpty());
        final Optional<ParameterOption> option = regions.stream().filter(o -> REGION_TASMANIA_KEY.equals(o.getValue()))
                .findFirst();
        assertTrue(option.isPresent());
        assertEquals(REGION_TASMANIA_KEY, option.get().getValue());
        assertEquals(REGION_TASMANIA_NAME, option.get().getLabel());
    }

    @Test
    public void testParsePropertyCityCorrectly() {
        assertTrue(ephemerisManager.cities.containsKey(REGION_TASMANIA_KEY));
        final List<ParameterOption> cities = ephemerisManager.cities.get(REGION_TASMANIA_KEY);
        assertFalse(cities.isEmpty());
        final Optional<ParameterOption> option = cities.stream().filter(o -> CITY_HOBARD_AREA_KEY.equals(o.getValue()))
                .findFirst();
        assertTrue(option.isPresent());
        assertEquals(CITY_HOBARD_AREA_KEY, option.get().getValue());
        assertEquals(CITY_HOBARD_AREA_NAME, option.get().getLabel());
    }

    @Test
    public void testConfigOptionProviderDaysetDefault() {
        final Collection<ParameterOption> options = ephemerisManager.getParameterOptions(CONFIG_URI, "dayset-weekend",
                null);
        assertNotNull(options);
        assertEquals(7, options.size());
    }

    @Test
    public void testConfigOptionProviderDaysetUS() {
        final Collection<ParameterOption> options = ephemerisManager.getParameterOptions(CONFIG_URI, "dayset-weekend",
                Locale.US);
        assertNotNull(options);
        assertEquals(7, options.size());
        assertEquals(Stream.of(new ParameterOption("MONDAY", "Monday"), new ParameterOption("TUESDAY", "Tuesday"),
                new ParameterOption("WEDNESDAY", "Wednesday"), new ParameterOption("THURSDAY", "Thursday"),
                new ParameterOption("FRIDAY", "Friday"), new ParameterOption("SATURDAY", "Saturday"),
                new ParameterOption("SUNDAY", "Sunday")).collect(Collectors.toList()), options);
    }

    @Test
    public void testConfigOptionProviderDaysetGerman() {
        final Collection<ParameterOption> options = ephemerisManager.getParameterOptions(CONFIG_URI, "dayset-weekend",
                Locale.GERMAN);
        assertNotNull(options);
        assertEquals(7, options.size());
        assertEquals(Stream.of(new ParameterOption("MONDAY", "Montag"), new ParameterOption("TUESDAY", "Dienstag"),
                new ParameterOption("WEDNESDAY", "Mittwoch"), new ParameterOption("THURSDAY", "Donnerstag"),
                new ParameterOption("FRIDAY", "Freitag"), new ParameterOption("SATURDAY", "Samstag"),
                new ParameterOption("SUNDAY", "Sonntag")).collect(Collectors.toList()), options);
    }

    @Test
    public void testConfigOptionProviderCountries() {
        final Collection<ParameterOption> options = ephemerisManager.getParameterOptions(CONFIG_URI, CONFIG_COUNTRY,
                null);
        assertNotNull(options);
        assertFalse(options.isEmpty());
        assertEquals(ephemerisManager.countries, options);
    }

    @Test
    public void testConfigOptionProviderRegionsAustria() {
        ephemerisManager.modified(Collections.singletonMap(CONFIG_COUNTRY, COUNTRY_AUSTRALIA_KEY));

        final Collection<ParameterOption> options = ephemerisManager.getParameterOptions(CONFIG_URI, CONFIG_REGION,
                null);
        assertNotNull(options);
        assertEquals(8, options.size());
        assertEquals(ephemerisManager.regions.get(COUNTRY_AUSTRALIA_KEY), options);
    }

    @Test
    public void testConfigOptionProviderRegionsGermany() {
        final Collection<ParameterOption> options = ephemerisManager.getParameterOptions(CONFIG_URI, CONFIG_REGION,
                null);
        assertNotNull(options);
        assertEquals(16, options.size());
        assertEquals(ephemerisManager.regions.get(Locale.GERMANY.getCountry().toLowerCase()), options);
    }

    @Test
    public void testConfigOptionProviderCitiesNorthRhineWestphalia() {
        ephemerisManager.modified(Collections.singletonMap(CONFIG_REGION, REGION_NORTHRHINEWESTPHALIA_KEY));

        final Collection<ParameterOption> options = ephemerisManager.getParameterOptions(CONFIG_URI, CONFIG_CITY, null);
        assertNull(options);
    }

    @Test
    public void testConfigOptionProviderCitiesTasmania() {
        ephemerisManager.modified(Collections.singletonMap(CONFIG_REGION, REGION_TASMANIA_KEY));

        final Collection<ParameterOption> options = ephemerisManager.getParameterOptions(CONFIG_URI, CONFIG_CITY, null);
        assertNotNull(options);
        assertFalse(options.isEmpty());
        assertEquals(ephemerisManager.cities.get(REGION_TASMANIA_KEY), options);
    }

    @Test
    public void testConfigOptionProviderCitiesBavaria() {
        ephemerisManager.modified(Collections.singletonMap(CONFIG_REGION, REGION_BAVARIA_KEY));

        final Collection<ParameterOption> options = ephemerisManager.getParameterOptions(CONFIG_URI, CONFIG_CITY, null);
        assertNotNull(options);
        assertFalse(options.isEmpty());
        assertEquals(ephemerisManager.cities.get(REGION_BAVARIA_KEY), options);
    }
}
