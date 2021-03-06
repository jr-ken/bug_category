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

import java.io.FileNotFoundException;
import java.net.MalformedURLException;
import java.net.URI;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.DayOfWeek;
import java.time.LocalDate;
import java.time.ZonedDateTime;
import java.time.format.TextStyle;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Stream;

import org.eclipse.jdt.annotation.NonNullByDefault;
import org.eclipse.jdt.annotation.Nullable;
import org.eclipse.smarthome.config.core.ConfigOptionProvider;
import org.eclipse.smarthome.config.core.ConfigurableService;
import org.eclipse.smarthome.config.core.ParameterOption;
import org.eclipse.smarthome.core.ephemeris.EphemerisManager;
import org.eclipse.smarthome.core.i18n.LocaleProvider;
import org.osgi.framework.Constants;
import org.osgi.service.component.annotations.Activate;
import org.osgi.service.component.annotations.Component;
import org.osgi.service.component.annotations.Modified;
import org.osgi.service.component.annotations.Reference;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import de.jollyday.Holiday;
import de.jollyday.HolidayManager;
import de.jollyday.ManagerParameter;
import de.jollyday.ManagerParameters;

/**
 * This service provides functionality around ephemeris services and is the central service to be used directly by
 * others.
 *
 * @author Gaël L'hopital - Initial contribution
 */
@Component(name = "org.openhab.ephemeris", property = { Constants.SERVICE_PID + "=org.openhab.ephemeris",
        ConfigurableService.SERVICE_PROPERTY_CATEGORY + "=system",
        ConfigurableService.SERVICE_PROPERTY_LABEL + "=Ephemeris",
        ConfigurableService.SERVICE_PROPERTY_DESCRIPTION_URI + "=" + EphemerisManagerImpl.CONFIG_URI })
@NonNullByDefault
public class EphemerisManagerImpl implements EphemerisManager, ConfigOptionProvider {
    private final Logger logger = LoggerFactory.getLogger(EphemerisManagerImpl.class);

    // constants for the configuration properties
    protected static final String CONFIG_URI = "system:ephemeris";
    private static final String CONFIG_DAYSET_PREFIX = "dayset-";
    private static final String CONFIG_DAYSET_WEEKEND = "weekend";
    public static final String CONFIG_COUNTRY = "country";
    public static final String CONFIG_REGION = "region";
    public static final String CONFIG_CITY = "city";

    private static final String PROPERTY_COUNTRY_DESCRIPTION_PREFIX = "country.description.";
    private static final String PROPERTY_COUNTRY_DESCRIPTION_DELIMITER = "\\.";

    final List<ParameterOption> countries = new ArrayList<>();
    final Map<String, List<ParameterOption>> regions = new HashMap<>();
    final Map<String, List<ParameterOption>> cities = new HashMap<>();

    private final Map<String, Set<DayOfWeek>> daysets = new HashMap<>();
    private final Map<Object, HolidayManager> holidayManagers = new HashMap<>();
    private final List<String> countryParameters = new ArrayList<>();

    private final LocaleProvider localeProvider;

    private @NonNullByDefault({}) String country;
    private @Nullable String region;

    @Activate
    public EphemerisManagerImpl(final @Reference LocaleProvider localeProvider) {
        this.localeProvider = localeProvider;

        final Bundle bundle = FrameworkUtil.getBundle(getClass());
        try (InputStream stream = bundle.getResource("jolliday/descriptions/country_descriptions.properties")
                .openStream()) {
            final Properties properties = new Properties();
            properties.load(stream);
            properties.forEach(this::parseProperty);
        } catch (IllegalArgumentException | IllegalStateException | IOException e) {
            logger.warn(
                    "The resource 'country_descriptions' could not be loaded properly! ConfigDescription options are not available.",
                    e);
        }
    }

    @Activate
    protected void activate(Map<String, Object> config) {
        modified(config);
    }

    @Modified
    protected void modified(Map<String, Object> config) {
        config.entrySet().stream().filter(e -> e.getKey().startsWith(CONFIG_DAYSET_PREFIX)).forEach(e -> {
            String[] setDefinition = e.getValue().toString().toUpperCase().split(",");
            String[] setNameParts = e.getKey().split("-");
            if (setDefinition.length > 0 && setNameParts.length > 1) {
                Set<DayOfWeek> dayset = new HashSet<>();
                Stream.of(setDefinition).forEach(day -> {
                    dayset.add(DayOfWeek.valueOf(day));
                });
                daysets.put(setNameParts[1], dayset);
            } else {
                logger.warn("Erroneous dayset definition {} : {}", e.getKey(), e.getValue());
            }
        });

        if (config.containsKey(CONFIG_COUNTRY)) {
            country = config.get(CONFIG_COUNTRY).toString();
        } else {
            country = localeProvider.getLocale().getCountry();
            logger.debug("Using system default country '{}' ", country);
        }

        if (config.containsKey(CONFIG_REGION)) {
            String region = config.get(CONFIG_REGION).toString();
            countryParameters.add(region);
            this.region = region;
        } else {
            this.region = null;
        }

        if (config.containsKey(CONFIG_CITY)) {
            countryParameters.add(config.get(CONFIG_CITY).toString());
        }
    }

    @Override
    public @Nullable Collection<ParameterOption> getParameterOptions(URI uri, String param, @Nullable Locale locale) {
        if (CONFIG_URI.equals(uri.toString())) {
            switch (param) {
                case CONFIG_COUNTRY:
                    return COUNTRIES;
                case CONFIG_REGION:
                    if (REGIONS.containsKey(country)) {
                        return REGIONS.get(country);
                    }
                case CONFIG_CITY:
                    if (region != null && CITIES.containsKey(region)) {
                        return CITIES.get(region);
                    }
                default:
                    if (param.startsWith(CONFIG_DAYSET_PREFIX)) {
                        Locale nullSafeLocale = locale == null ? localeProvider.getLocale() : locale;
                        final List<ParameterOption> options = new ArrayList<>();
                        for (DayOfWeek day : DayOfWeek.values()) {
                            ParameterOption option = new ParameterOption(day.name(),
                                    day.getDisplayName(TextStyle.FULL, nullSafeLocale));
                            options.add(option);
                        }
                        return options;
                    }
                    break;
            }
        }
        return null;
    }

    private HolidayManager getHolidayManager(Object managerKey) {
        return holidayManagers.computeIfAbsent(managerKey, key -> {
            final ManagerParameter parameters = managerKey.getClass() == String.class
                    ? ManagerParameters.create((String) managerKey)
                    : ManagerParameters.create((URL) managerKey);
            return HolidayManager.getInstance(parameters);
        });
    }

    private Optional<Holiday> getHoliday(ZonedDateTime date) {
        HolidayManager manager = getHolidayManager(country);
        LocalDate localDate = date.toLocalDate();

        Set<Holiday> holidays = manager.getHolidays(localDate, localDate, countryParameters.toArray(new String[0]));
        return holidays.isEmpty() ? Optional.empty() : Optional.of(holidays.iterator().next());
    }

    private boolean isBankHoliday(ZonedDateTime date) {
        Optional<Holiday> holiday = getHoliday(date);
        return holiday.isPresent();
    }

    @Override
    public boolean isBankHoliday(int offset) {
        return isBankHoliday(ZonedDateTime.now().plusDays(offset));
    }

    @Override
    public boolean isBankHoliday(int offset, String filename) throws FileNotFoundException {
        Optional<String> holiday = getHolidayUserFile(ZonedDateTime.now().plusDays(offset), filename);
        return holiday.isPresent();
    }

    @Override
    public boolean isWeekend(int offset) {
        return isWeekend(ZonedDateTime.now().plusDays(offset));
    }

    private boolean isWeekend(ZonedDateTime date) {
        return isInDayset(CONFIG_DAYSET_WEEKEND, date);
    }

    @Override
    public boolean isInDayset(String daysetName, int offset) {
        return isInDayset(daysetName, ZonedDateTime.now().plusDays(offset));
    }

    private boolean isInDayset(String daysetName, ZonedDateTime date) {
        if (daysets.containsKey(daysetName)) {
            DayOfWeek dow = date.getDayOfWeek();
            return daysets.get(daysetName).contains(dow);
        } else {
            logger.warn("This dayset is not configured : {}", daysetName);
            return false;
        }
    }

    private String getBankHolidayName(ZonedDateTime date) {
        Optional<Holiday> holiday = getHoliday(date);
        return holiday.map(p -> p.getDescription(localeProvider.getLocale())).orElse(null);
    }

    @Override
    public @Nullable String getBankHolidayName(int offset) {
        return getBankHolidayName(ZonedDateTime.now().plusDays(offset));
    }

    private Optional<String> getHolidayUserFile(ZonedDateTime date, String filename) throws FileNotFoundException {
        if (Files.exists(Paths.get(filename))) {
            try {
                URL url = new URL("file:" + filename);
                Set<Holiday> days = getHolidayManager(url).getHolidays(date.toLocalDate(), date.toLocalDate());
                return days.isEmpty() ? Optional.empty() : Optional.of(days.iterator().next().getPropertiesKey());
            } catch (MalformedURLException e) {
                throw new FileNotFoundException(e.getMessage());
            }
        } else {
            throw new FileNotFoundException();
        }
    }

    @Override
    public @Nullable String getBankHolidayName(int offset, String filename) throws FileNotFoundException {
        Optional<String> holiday = getHolidayUserFile(ZonedDateTime.now().plusDays(offset), filename);
        return holiday.isPresent() ? holiday.get() : null;
    }

    /**
     * Parses each entry of a properties list loaded from 'jolliday/country_names.properties'. The content of that files
     * has been copied from http://jollyday.sourceforge.net/names_country_default.html and contains values in the
     * following format - some examples:
     *
     * country.description.at = Austria
     * country.description.au = Australia
     * country.description.au.sa = South Australia
     * country.description.au.tas = Tasmania
     * country.description.au.tas.ho = Hobard Area
     * country.description.au.tas.nh = Non-Hobard Area
     *
     * "at" and "au" are keys of countries
     * "sa" and "tas" are keys of regions inside the country "au"
     * "ho" and "nh" are kess of cities or areas inside the region "tas"
     *
     * @param key key of the property will be parsed
     * @param value value of the property will be used as name
     * @throws IllegalArgumentException if the property could not be parsed properly
     */
    static void parseProperty(Object key, Object value) throws IllegalArgumentException {
        final String property = key.toString().replace(PROPERTY_COUNTRY_DESCRIPTION_PREFIX, "");
        final String[] parts = property.split(PROPERTY_COUNTRY_DESCRIPTION_DELIMITER);
        final String name = value.toString();
        final String part;
        final ParameterOption option;
        switch (parts.length) {
            case 1:
                COUNTRIES.add(new ParameterOption(getValidPart(parts[0]), name));
                break;
            case 2:
                part = getValidPart(parts[0]);
                option = new ParameterOption(getValidPart(parts[1]), name);
                if (REGIONS.containsKey(part)) {
                    REGIONS.get(part).add(option);
                } else {
                    REGIONS.put(part, Arrays.asList(option));
                }
                break;
            case 3:
                part = getValidPart(parts[1]);
                option = new ParameterOption(getValidPart(parts[2]), name);
                if (CITIES.containsKey(part)) {
                    CITIES.get(part).add(option);
                } else {
                    CITIES.put(part, Arrays.asList(option));
                }
                break;
            default:
                throw new IllegalArgumentException(String.format("Unable to parse property '%s = %s'.", key, value));
        }
    }

    private static String getValidPart(String part) {
        final String subject = part.trim();
        if (!subject.isEmpty()) {
            return subject;
        } else {
            throw new IllegalArgumentException("Unable to parse property - token is empty.");
        }
    }
}
