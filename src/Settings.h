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

    // Per-priority history: for each setting key, a map of priority (int) → stored value.
    // All writes at any priority level are recorded here; the active *Settings maps hold the current winning value.
    std::map<PairString, std::map<int, std::string>> stringSettingHistory;
    std::map<PairString, std::map<int, double>>      doubleSettingHistory;
    std::map<PairString, std::map<int, int>>         integerSettingHistory;
    std::map<PairString, std::map<int, bool>>        booleanSettingHistory;

    // The highest priority at which the active value was written (0 = default).
    std::map<PairString, int> settingActivePriority;

    std::map<PairString, PairString> settingGroupDescriptions;
    std::map<PairString, std::string> settingDescriptions;

    std::map<PairString, E_SettingType> settingTypes;
    std::map<PairString, bool> settingIsPrivate;
    std::map<PairString, PairDouble> settingBounds;
    std::map<PairString, bool> settingEnums;

    // Splits "Category.Name.Sub" at the first dot into {category, remainder}.
    // E.g. "Dual.MIP.Solver" -> {"Dual", "MIP.Solver"}
    static PairString splitKey(const std::string& settingName)
    {
        auto dot = settingName.find('.');
        if(dot == std::string::npos)
            return { settingName, "" };
        return { settingName.substr(0, dot), settingName.substr(dot + 1) };
    }

    using TupleStringPairInt = std::tuple<std::string, std::string, int>;
    std::map<TupleStringPairInt, std::string> enumDescriptions;

public:
    bool settingsInitialized = false;

    Settings(OutputPtr outputPtr);

    ~Settings();

    template <typename T>
    void updateSetting(std::string name, std::string category, T value,
        E_SettingPriority priority = E_SettingPriority::SolverInternal);

    // Single-string form: updateSetting("Category.Name", value [, priority])
    template <typename T>
    void updateSetting(std::string settingName, T value,
        E_SettingPriority priority = E_SettingPriority::SolverInternal)
    {
        auto [cat, name] = splitKey(settingName);
        updateSetting(name, cat, value, priority);
    }

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

    // Single-string form: getSetting<T>("Category.Name")
    template <typename T> T getSetting(std::string settingName)
    {
        auto [cat, name] = splitKey(settingName);
        return getSetting<T>(name, cat);
    }

    std::string getSettingDescription(std::string name, std::string category)
    {
        return settingDescriptions.at(PairString(category, name));
    }

    std::string getSettingDescription(std::string settingName)
    {
        auto [cat, name] = splitKey(settingName);
        return getSettingDescription(name, cat);
    }

    PairDouble getSettingBounds(std::string name, std::string category)
    {
        return settingBounds.at(PairString(category, name));
    }

    PairDouble getSettingBounds(std::string settingName)
    {
        auto [cat, name] = splitKey(settingName);
        return getSettingBounds(name, cat);
    }

    void createSetting(std::string settingName, std::string value, std::string description, bool isPrivate = false);

    void createSetting(std::string settingName, int value, std::string description,
        double minVal = 0, double maxVal = SHOT_INT_MAX, bool isPrivate = false);

    void createSetting(std::string settingName, double value, std::string description,
        double minVal = SHOT_DBL_MIN, double maxVal = SHOT_DBL_MAX, bool isPrivate = false);

    void createSetting(std::string settingName, int value, std::string description,
        VectorString enumDesc, int startValue = 0, bool isPrivate = false);

    void createSetting(std::string settingName, bool value, std::string description, bool isPrivate = false);

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

    // Priority query methods
    E_SettingPriority getSettingPriority(std::string name, std::string category);
    E_SettingPriority getSettingPriority(std::string settingName)
    {
        auto [cat, name] = splitKey(settingName);
        return getSettingPriority(name, cat);
    }

    bool isSettingAtDefault(std::string name, std::string category);
    bool isSettingAtDefault(std::string settingName)
    {
        auto [cat, name] = splitKey(settingName);
        return isSettingAtDefault(name, cat);
    }

    VectorString getSettingsAtPriority(E_SettingPriority priority);

    bool hasSettingAtPriority(std::string name, std::string category, E_SettingPriority priority);
    bool hasSettingAtPriority(std::string settingName, E_SettingPriority priority)
    {
        auto [cat, name] = splitKey(settingName);
        return hasSettingAtPriority(name, cat, priority);
    }

    std::vector<E_SettingPriority> getSettingPriorityHistory(std::string name, std::string category);
    std::vector<E_SettingPriority> getSettingPriorityHistory(std::string settingName)
    {
        auto [cat, name] = splitKey(settingName);
        return getSettingPriorityHistory(name, cat);
    }

    template <typename T>
    T getSettingAtPriority(std::string name, std::string category, E_SettingPriority priority)
    {
        using value_type
#ifndef _MSC_VER
            [[maybe_unused]]
#endif
            = typename std::enable_if<std::is_same<std::string, T>::value || std::is_same<double, T>::value
                    || std::is_same<int, T>::value || std::is_same<bool, T>::value,
                T>::type;

        PairString key = make_pair(category, name);
        int prio = static_cast<int>(priority);

        if(settingTypes.find(key) == settingTypes.end())
        {
            output->outputError("Cannot get setting " + category + "." + name + " since it has not been defined.");
            throw SettingKeyNotFoundException(name, category);
        }

        if constexpr(std::is_same_v<T, std::string>)
        {
            auto it = stringSettingHistory.find(key);
            if(it == stringSettingHistory.end() || it->second.find(prio) == it->second.end())
                throw SettingKeyNotFoundException(name, category);
            return it->second.at(prio);
        }
        else if constexpr(std::is_same_v<T, int>)
        {
            auto it = integerSettingHistory.find(key);
            if(it == integerSettingHistory.end() || it->second.find(prio) == it->second.end())
                throw SettingKeyNotFoundException(name, category);
            return it->second.at(prio);
        }
        else if constexpr(std::is_same_v<T, double>)
        {
            auto it = doubleSettingHistory.find(key);
            if(it == doubleSettingHistory.end() || it->second.find(prio) == it->second.end())
                throw SettingKeyNotFoundException(name, category);
            return it->second.at(prio);
        }
        else if constexpr(std::is_same_v<T, bool>)
        {
            auto it = booleanSettingHistory.find(key);
            if(it == booleanSettingHistory.end() || it->second.find(prio) == it->second.end())
                throw SettingKeyNotFoundException(name, category);
            return it->second.at(prio);
        }
    }

    template <typename T>
    T getSettingAtPriority(std::string settingName, E_SettingPriority priority)
    {
        auto [cat, name] = splitKey(settingName);
        return getSettingAtPriority<T>(name, cat, priority);
    }

    bool readSettingsFromOSoL(std::string osol);
    bool readSettingsFromString(std::string options);

};
} // namespace SHOT
