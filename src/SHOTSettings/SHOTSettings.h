/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Shared.h"
#include "tinyxml2.h"

namespace SHOT
{
class Settings
{
private:
    void updateSettingBase(PairString key, std::string value);

    Output* output;

    std::map<PairString, std::string> settings;

    typedef std::map<PairString, std::string>::iterator SettingsIter;
    SettingsIter settingsIterator;

    std::map<PairString, std::string> settingDescriptions;
    std::map<PairString, E_SettingType> settingTypes;
    std::map<PairString, bool> settingIsPrivate;
    std::map<PairString, bool> settingIsDefaultValue;
    std::map<PairString, PairDouble> settingBounds;
    std::map<PairString, bool> settingEnums;

    typedef std::tuple<std::string, std::string, int> TupleStringPairInt;

    std::map<TupleStringPairInt, std::string> enumDescriptions;

    typedef std::map<TupleStringPairInt, std::string>::iterator EnumDescriptionIter;

public:
    Settings(OutputPtr outputPtr)
    {
        output = outputPtr.get();
        settingsInitialized = false;
    }

    ~Settings() {}

    bool settingsInitialized = false;

    void readSettingsFromOSoL(std::string osol);
    void readSettingsFromString(std::string options);

    std::string getSettingsAsOSoL();
    std::string getSettingsAsString(bool showUnchanged, bool showDescriptions);

    // String settings
    void createSetting(
        std::string name, std::string category, std::string value, std::string description, bool isPrivate);
    void createSetting(std::string name, std::string category, std::string value, std::string description);
    void updateSetting(std::string name, std::string category, std::string value);
    std::string getStringSetting(std::string name, std::string category);

    // Integer settings
    void createSetting(std::string name, std::string category, int value, std::string description);
    void createSetting(
        std::string name, std::string category, int value, std::string description, double minVal, double maxVal);
    void createSetting(std::string name, std::string category, int value, std::string description, double minVal,
        double maxVal, bool isPrivate);
    void updateSetting(std::string name, std::string category, int value);
    int getIntSetting(std::string name, std::string category);

    // Boolean settings
    void createSetting(std::string name, std::string category, bool value, std::string description, bool isPrivate);
    void createSetting(std::string name, std::string category, bool value, std::string description);
    void updateSetting(std::string name, std::string category, bool value);
    bool getBoolSetting(std::string name, std::string category);

    // Enum setting
    void createSetting(std::string name, std::string category, int value, std::string description,
        VectorString enumDescriptions, bool isPrivate);
    void createSetting(
        std::string name, std::string category, int value, std::string description, VectorString enumDescriptions);
    std::string getEnumDescriptionList(std::string name, std::string category);
    std::string getEnumDescription(std::string name, std::string category);

    // Double settings
    void createSetting(std::string name, std::string category, double value, std::string description, double minVal,
        double maxVal, bool isPrivate);
    void createSetting(std::string name, std::string category, double value, std::string description);
    void createSetting(
        std::string name, std::string category, double value, std::string description, double minVal, double maxVal);
    void updateSetting(std::string name, std::string category, double value);
    double getDoubleSetting(std::string name, std::string category);
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