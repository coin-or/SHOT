/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <map>

#include "Enums.h"
#include "Output.h"
#include "Structs.h"

namespace SHOT
{

class SettingKeyNotFoundException : public std::runtime_error
{
public:
    SettingKeyNotFoundException(const std::string& key, const std::string& category)
        : std::runtime_error(fmt::format("Setting {}.{} not found!", category, key))
    {
    }
};

class SettingSetWrongTypeException : public std::runtime_error
{
public:
    SettingSetWrongTypeException(const std::string& key, const std::string& category)
        : std::runtime_error(fmt::format("Cannot set setting {}.{} since value is of the wrong type!", category, key))
    {
    }
};

class SettingGetWrongTypeException : public std::runtime_error
{
public:
    SettingGetWrongTypeException(const std::string& key, const std::string& category)
        : std::runtime_error(fmt::format("Cannot get setting {}.{} since value is of the wrong type!", category, key))
    {
    }
};

class SettingOutsideBoundsException : public std::runtime_error
{
public:
    SettingOutsideBoundsException(const std::string& key, const std::string& category, const double& value,
        const double& minVal, const double& maxVal)
        : std::runtime_error(fmt::format(
              "The value {} of setting {}.{} is not in interval [{},{}]!", value, category, key, minVal, maxVal))
    {
    }
};

class DllExport Settings
{
private:
    template <typename T>
    void createBaseSetting(
        std::string name, std::string category, T value, std::string description, bool isPrivate = false);

    using OutputPtr = std::shared_ptr<Output>;
    OutputPtr output;

    using PairString = std::pair<std::string, std::string>;
    using PairDouble = std::pair<double, double>;
    using VectorString = std::vector<std::string>;

    std::map<PairString, std::string> stringSettings;
    std::map<PairString, double> doubleSettings;
    std::map<PairString, int> integerSettings;
    std::map<PairString, bool> booleanSettings;

    std::map<PairString, PairString> settingGroupDescriptions;
    std::map<PairString, std::string> settingDescriptions;

    std::map<PairString, E_SettingType> settingTypes;
    std::map<PairString, bool> settingIsPrivate;
    std::map<PairString, bool> settingIsDefaultValue;
    std::map<PairString, PairDouble> settingBounds;
    std::map<PairString, bool> settingEnums;

    using TupleStringPairInt = std::tuple<std::string, std::string, int>;
    std::map<TupleStringPairInt, std::string> enumDescriptions;

public:
    bool settingsInitialized = false;

    Settings(OutputPtr outputPtr);

    ~Settings();

    template <typename T> void updateSetting(std::string name, std::string category, T value);

    // template <typename T> T getSetting(std::string name, std::string category);

    template <typename T> T getSetting(std::string name, std::string category)
    {
        // Check that setting is of the correct type
        using value_type
#ifndef _MSC_VER
            [[maybe_unused]]
#endif
            = typename std::enable_if<std::is_same<std::string, T>::value || std::is_same<double, T>::value
                    || std::is_same<int, T>::value || std::is_same<bool, T>::value,
                T>::type;

        PairString key = make_pair(category, name);

        typename std::map<PairString, T>::iterator value;
        typename std::map<PairString, T>::iterator end;

        if constexpr(std::is_same_v<T, std::string>)
        {
            value = stringSettings.find(key);
            end = stringSettings.end();
        }
        else if constexpr(std::is_same_v<T, int>)
        {
            value = integerSettings.find(key);
            end = integerSettings.end();
        }
        else if constexpr(std::is_same_v<T, double>)
        {
            value = doubleSettings.find(key);
            end = doubleSettings.end();
        }
        else if constexpr(std::is_same_v<T, bool>)
        {
            value = booleanSettings.find(key);
            end = booleanSettings.end();
        }

        if(value == end)
        {
            output->outputError("Cannot get setting " + category + "." + name + " since it has not been defined.");

            throw SettingKeyNotFoundException(name, category);
        }

        return (value->second);
    }

    std::string getSettingDescription(std::string name, std::string category)
    {
        return settingDescriptions.at(PairString(category, name));
    }

    PairDouble getSettingBounds(std::string name, std::string category)
    {
        return settingBounds.at(PairString(category, name));
    }

    void createSetting(
        std::string name, std::string category, std::string value, std::string description, bool isPrivate = false);

    void createSetting(std::string name, std::string category, int value, std::string description, double minVal = 0,
        double maxVal = SHOT_INT_MAX, bool isPrivate = false);

    void createSetting(std::string name, std::string category, double value, std::string description,
        double minVal = SHOT_DBL_MIN, double maxVal = SHOT_DBL_MAX, bool isPrivate = false);

    void createSetting(std::string name, std::string category, int value, std::string description,
        VectorString enumDesc, int startValue = 0, bool isPrivate = false);

    void createSetting(
        std::string name, std::string category, bool value, std::string description, bool isPrivate = false);

    void createSettingGroup(std::string mainLevel, std::string subLevel, std::string header, std::string description)
    {
        settingGroupDescriptions.emplace(make_pair(mainLevel, subLevel), make_pair(header, description));
    }

    PairString getCategoryDescription(std::string category)
    {
        return settingGroupDescriptions.at(PairString(category, ""));
    }

    std::string getEnumDescriptionList(std::string name, std::string category);
    std::string getEnumDescriptionListMarkup(std::string name, std::string category);
    std::vector<std::pair<int, std::string>> getEnumDescription(std::string name, std::string category);

    std::string getSettingsAsOSoL();
    std::string getSettingsAsString(bool showUnchanged, bool showDescriptions);
    std::string getSettingsAsMarkup();

    VectorString getChangedSettings();
    VectorString getSettingIdentifiers(E_SettingType type);
    VectorPairString getSettingSplitIdentifiers(E_SettingType type);

    bool readSettingsFromOSoL(std::string osol);
    bool readSettingsFromString(std::string options);
};
} // namespace SHOT
