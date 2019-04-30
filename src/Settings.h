/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <algorithm>
#include <map>

#include "tinyxml2.h"

#include "../Enums.h"
#include "../Output.h"

namespace SHOT
{
class Settings
{
private:
    template <typename T>
    void createBaseSetting(
        std::string name, std::string category, T value, std::string description, bool isPrivate = false);

    Output* output;
    typedef std::shared_ptr<Output> OutputPtr;

    typedef std::pair<std::string, std::string> PairString;
    typedef std::pair<double, double> PairDouble;
    typedef std::vector<std::string> VectorString;

    std::map<PairString, std::string> stringSettings;
    std::map<PairString, double> doubleSettings;
    std::map<PairString, int> integerSettings;
    std::map<PairString, bool> booleanSettings;

    std::map<PairString, std::string> settingDescriptions;
    std::map<PairString, E_SettingType> settingTypes;
    std::map<PairString, bool> settingIsPrivate;
    std::map<PairString, bool> settingIsDefaultValue;
    std::map<PairString, PairDouble> settingBounds;
    std::map<PairString, bool> settingEnums;

    typedef std::tuple<std::string, std::string, int> TupleStringPairInt;
    std::map<TupleStringPairInt, std::string> enumDescriptions;

public:
    bool settingsInitialized = false;

    Settings(OutputPtr outputPtr) { output = outputPtr.get(); }

    ~Settings() = default;

    template <typename T> void updateSetting(std::string name, std::string category, T value);

    template <typename T> T getSetting(std::string name, std::string category);

    void createSetting(
        std::string name, std::string category, std::string value, std::string description, bool isPrivate = false);

    void createSetting(std::string name, std::string category, int value, std::string description,
        double minVal = std::numeric_limits<double>::lowest(), double maxVal = std::numeric_limits<double>::max(),
        bool isPrivate = false);

    void createSetting(std::string name, std::string category, double value, std::string description,
        double minVal = std::numeric_limits<double>::lowest(), double maxVal = std::numeric_limits<double>::max(),
        bool isPrivate = false);

    void createSetting(std::string name, std::string category, int value, std::string description,
        VectorString enumDesc, bool isPrivate = false);

    void createSetting(
        std::string name, std::string category, bool value, std::string description, bool isPrivate = false);

    std::string getEnumDescriptionList(std::string name, std::string category);
    std::string getEnumDescription(std::string name, std::string category);

    std::string getSettingsAsOSoL();
    std::string getSettingsAsString(bool showUnchanged, bool showDescriptions);

    bool readSettingsFromOSoL(std::string osol);
    bool readSettingsFromString(std::string options);
};

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
}; // namespace SHOT