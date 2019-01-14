/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Shared.h"

#include "OSOption.h"
#include "OSoLWriter.h"
#include "OSoLReader.h"

namespace SHOT
{
class Settings
{
private:
    void updateSettingBase(std::pair<std::string, std::string> key, std::string value);

    Output* output;

public:
    Settings(OutputPtr outputPtr)
    {
        output = outputPtr.get();
        settingsInitialized = false;
    }

    ~Settings() {}

    bool settingsInitialized = false;

    void readSettingsFromOSoL(std::string osol);
    void readSettingsFromOSOption(OSOption* options);
    void readSettingsFromGAMSOptFormat(std::string options);

    std::string getSettingsInOSolFormat();
    std::string getSettingsAsString();
    std::string getUpdatedSettingsAsString();
    std::unique_ptr<OSOption> getSettingsAsOSOption();
    std::string getSettingsInGAMSOptFormat();
    std::string getSettingsInGAMSOptFormat(bool includeDescriptions);

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
        : std::runtime_error(str(
              boost::format("Exception: Setting with <key,category> std::pair <%1%,%2%> not found!") % key % category))
    {
    }
};

class SettingSetWrongTypeException : public std::runtime_error
{
public:
    SettingSetWrongTypeException(const std::string& key, const std::string& category)
        : std::runtime_error(
              str(boost::format("Exception: Cannot set setting with <key,category> std::pair <%1%,%2%>, wrong type!")
                  % key % category))
    {
    }
};

class SettingGetWrongTypeException : public std::runtime_error
{
public:
    SettingGetWrongTypeException(const std::string& key, const std::string& category)
        : std::runtime_error(
              str(boost::format("Exception: Cannot get setting with <key,category> std::pair <%1%,%2%>, wrong type!")
                  % key % category))
    {
    }
};

class SettingOutsideBoundsException : public std::runtime_error
{
public:
    SettingOutsideBoundsException(const std::string& key, const std::string& category, const double& value,
        const double& minVal, const double& maxVal)
        : std::runtime_error(str(boost::format("Exception: The value %1% of setting with <key,category> std::pair "
                                               "<%2%,%3%> is not between %4% and %5%!")
              % value % key % category % minVal % maxVal))
    {
    }
};
}; // namespace SHOT