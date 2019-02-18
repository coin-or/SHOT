/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "SHOTSettings.h"

namespace SHOT
{

void Settings::updateSettingBase(PairString key, std::string value)
{
    std::string oldvalue = settings[key];

    if(oldvalue == value)
    {
        output->outputTrace(
            "Setting " + key.first + "." + key.second + " not updated. Same value " + oldvalue + " given.");
        return;
    }
    else
    {
        settings[key] = value;
        settingIsDefaultValue[key] = false;

        output->outputTrace(
            "Setting " + key.first + "." + key.second + " = " + oldvalue + " updated. New value = " + value + ".");
    }
}

void Settings::createSetting(std::string name, std::string category, std::string value, std::string description)
{
    createSetting(name, category, value, description, false);
}

void Settings::createSetting(
    std::string name, std::string category, std::string value, std::string description, bool isPrivate)
{
    PairString key = make_pair(category, name);
    settings[key] = value;
    settingDescriptions[key] = description;
    settingTypes[key] = E_SettingType::String;
    std::string test = name;
    settingIsPrivate[key] = isPrivate;
    settingIsDefaultValue[key] = true;

    output->outputTrace("Setting " + category + "." + name + " = " + value + " created.");
}

void Settings::updateSetting(std::string name, std::string category, std::string value)
{
    PairString key = make_pair(category, name);
    settingsIterator = settings.find(key);

    if(settingsIterator == settings.end())
    {
        output->outputError("Cannot update setting " + category + "." + name + " since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(settingTypes[key] != E_SettingType::String)
    {
        output->outputError(
            "Cannot update setting " + category + "." + name + " since it is of the wrong type. (Expected string).");

        throw SettingSetWrongTypeException(name, category);
    }

    updateSettingBase(key, value);
}

std::string Settings::getStringSetting(std::string name, std::string category)
{
    PairString key = make_pair(category, name);
    settingsIterator = settings.find(key);

    if(settingsIterator == settings.end())
    {
        output->outputError("Cannot get setting " + category + "." + name + " since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(settingTypes[key] != E_SettingType::String)
    {
        output->outputError(
            "Cannot get value of setting " + category + "." + name + " as string: Wrong type requested!");

        throw SettingGetWrongTypeException(name, category);
    }

    return settings[key];
}

// Integer settings ==============================================================

void Settings::createSetting(std::string name, std::string category, int value, std::string description, double minVal,
    double maxVal, bool isPrivate)
{
    PairString key = make_pair(category, name);
    settings[key] = std::to_string(value);
    settingDescriptions[key] = description;
    settingTypes[key] = E_SettingType::Integer;
    settingBounds[key] = std::make_pair(minVal, maxVal);
    settingIsPrivate[key] = isPrivate;
    settingIsDefaultValue[key] = true;

    output->outputTrace("Setting " + category + "." + name + " = " + std::to_string(value) + " created.");
}

void Settings::createSetting(
    std::string name, std::string category, int value, std::string description, double minVal, double maxVal)
{
    Settings::createSetting(name, category, value, description, minVal, maxVal, false);
}

void Settings::createSetting(std::string name, std::string category, int value, std::string description)
{
    Settings::createSetting(name, category, value, description, SHOT_DBL_MIN, SHOT_DBL_MAX, false);
}

void Settings::updateSetting(std::string name, std::string category, int value)
{
    PairString key = make_pair(category, name);
    settingsIterator = settings.find(key);

    if(settingsIterator == settings.end())
    {
        output->outputError("Cannot update setting " + category + "." + name + " since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(settingTypes[key] != E_SettingType::Integer)
    {
        output->outputError("Cannot set value of setting " + category + "." + name + " as integer: Wrong type!");

        throw SettingSetWrongTypeException(name, category);
    }

    if(settingBounds[key].first > value || settingBounds[key].second < value)
    {
        output->outputError("Cannot update setting " + category + "." + name + ": Not in interval ["
            + std::to_string(settingBounds[key].first) + "," + std::to_string(settingBounds[key].second) + "].");

        throw SettingOutsideBoundsException(
            name, category, (double)value, settingBounds[key].first, settingBounds[key].second);
    }

    updateSettingBase(key, std::to_string(value));
}

int Settings::getIntSetting(std::string name, std::string category)
{
    PairString key = make_pair(category, name);
    settingsIterator = settings.find(key);

    if(settingsIterator == settings.end())
    {
        output->outputError("Cannot get setting " + category + "." + name + " since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(settingTypes[key] != E_SettingType::Integer)
    {
        output->outputError(
            "Cannot get value of setting " + category + "." + name + " as integer: Wrong type requested!");

        throw SettingGetWrongTypeException(name, category);
    }

    int intval = std::stoi(settings[key]);
    return intval;
}

// Boolean settings ==============================================================

void Settings::createSetting(
    std::string name, std::string category, bool value, std::string description, bool isPrivate)
{
    PairString key = make_pair(category, name);
    settings[key] = std::to_string(value);
    settingDescriptions[key] = description;
    settingTypes[key] = E_SettingType::Boolean;
    settingIsPrivate[key] = isPrivate;
    settingIsDefaultValue[key] = true;

    output->outputTrace("Setting " + category + "." + name + " = " + std::to_string(value) + " created.");
}

void Settings::createSetting(std::string name, std::string category, bool value, std::string description)
{
    Settings::createSetting(name, category, value, description, false);
}

void Settings::updateSetting(std::string name, std::string category, bool value)
{
    PairString key = make_pair(category, name);
    settingsIterator = settings.find(key);

    if(settingsIterator == settings.end())
    {
        output->outputError("Cannot update setting " + category + "." + name + " since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(settingTypes[key] != E_SettingType::Boolean)
    {
        output->outputError("Cannot set value of setting " + category + "." + name + " as bool: Wrong type!");

        throw SettingSetWrongTypeException(name, category);
    }

    std::string newvalue = std::to_string(value);

    updateSettingBase(key, newvalue);
}

bool Settings::getBoolSetting(std::string name, std::string category)
{
    PairString key = make_pair(category, name);
    settingsIterator = settings.find(key);

    if(settingsIterator == settings.end())
    {
        output->outputError("Cannot get setting " + category + "." + name + " since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(settingTypes[key] != E_SettingType::Boolean)
    {
        output->outputError(
            "Cannot get value of setting " + category + "." + name + " as boolean: Wrong type requested!");

        throw SettingGetWrongTypeException(name, category);
    }

    bool boolval = (settings[key] != "0");
    return boolval;
}

// Enum settings ==================================================================

void Settings::createSetting(
    std::string name, std::string category, int value, std::string description, VectorString enumDesc, bool isPrivate)
{
    createSetting(name, category, value, description, SHOT_DBL_MIN, SHOT_DBL_MAX, isPrivate);

    for(int i = 0; i < enumDesc.size(); i++)
    {
        enumDescriptions[make_tuple(category, name, i)] = enumDesc.at(i);

        output->outputTrace(" Enum value " + std::to_string(i) + ": " + enumDesc.at(i));
    }

    settingEnums[make_pair(category, name)] = true;
}

void Settings::createSetting(
    std::string name, std::string category, int value, std::string description, VectorString enumDescriptions)
{
    Settings::createSetting(name, category, value, description, enumDescriptions, false);
}

std::string Settings::getEnumDescriptionList(std::string name, std::string category)
{
    std::stringstream desc;

    for(EnumDescriptionIter iterator = enumDescriptions.begin(); iterator != enumDescriptions.end(); iterator++)
    {
        std::tuple<std::string, std::string, int> t = iterator->first;

        if(name == std::get<1>(t) && category == std::get<0>(t))
        {
            desc << std::get<2>(t) << ": " << iterator->second << ". ";
        }
    }

    return desc.str();
}

std::string Settings::getEnumDescription(std::string name, std::string category)
{
    PairString key = make_pair(category, name);
    settingsIterator = settings.find(key);

    if(settingsIterator == settings.end())
    {
        output->outputError("Cannot update setting " + category + "." + name + " since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(settingTypes[key] != E_SettingType::Integer)
    {
        output->outputError(
            "Cannot get value of setting " + category + "." + name + " as integer: Wrong type requested!");

        throw SettingGetWrongTypeException(name, category);
    }

    int intval = std::stoi(settings[key]);
    std::tuple<std::string, std::string, int> tpl = make_tuple(category, name, intval);

    return enumDescriptions[tpl];
}

// Double settings ================================================================

void Settings::createSetting(std::string name, std::string category, double value, std::string description)
{
    Settings::createSetting(name, category, value, description, SHOT_DBL_MIN, SHOT_DBL_MAX, false);
}

void Settings::createSetting(
    std::string name, std::string category, double value, std::string description, double minVal, double maxVal)
{
    Settings::createSetting(name, category, value, description, SHOT_DBL_MIN, SHOT_DBL_MAX, false);
}

void Settings::createSetting(std::string name, std::string category, double value, std::string description,
    double minVal, double maxVal, bool isPrivate)
{
    PairString key = make_pair(category, name);
    settings[key] = std::to_string(value);
    settingDescriptions[key] = description;
    settingTypes[key] = E_SettingType::Double;
    settingBounds[key] = std::make_pair(minVal, maxVal);
    settingIsPrivate[key] = isPrivate;
    settingIsDefaultValue[key] = true;

    output->outputTrace("Setting " + category + "." + name + " = " + std::to_string(value) + " created.");
}

void Settings::updateSetting(std::string name, std::string category, double value)
{
    PairString key = make_pair(category, name);
    settingsIterator = settings.find(key);

    if(settingsIterator == settings.end())
    {
        output->outputError("Cannot update setting " + category + "." + name + " since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(settingTypes[key] != E_SettingType::Double)
    {
        output->outputError("Cannot set value of setting " + category + "." + name + " as double: Wrong type!");

        throw SettingSetWrongTypeException(name, category);
    }

    if(settingBounds[key].first > value || settingBounds[key].second < value)
    {
        output->outputError("Cannot update setting " + category + "." + name + ": Not in interval ["
            + std::to_string(settingBounds[key].first) + "," + std::to_string(settingBounds[key].second) + "].");

        throw SettingOutsideBoundsException(name, category, value, settingBounds[key].first, settingBounds[key].second);
    }

    std::string newvalue = std::to_string(value);

    updateSettingBase(key, newvalue);
}

double Settings::getDoubleSetting(std::string name, std::string category)
{
    PairString key = make_pair(category, name);
    settingsIterator = settings.find(key);

    if(settingsIterator == settings.end())
    {
        output->outputError("Cannot get setting " + category + "." + name + " since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(settingTypes[key] != E_SettingType::Double)
    {
        output->outputError(
            "Cannot get value of setting " + category + "." + name + " as double: Wrong type requested!");

        throw SettingGetWrongTypeException(name, category);
    }

    double intval = std::stod(settings[key]);
    return intval;
}

std::string Settings::getSettingsAsOSoL()
{
    using namespace tinyxml2;

    XMLDocument osolDocument;

    auto osolNode = osolDocument.NewElement("osol");
    osolNode->SetAttribute("xmlns", "os.optimizationservices.org");
    osolNode->SetAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
    osolNode->SetAttribute(
        "xmlns:schemaLocation", "os.optimizationservices.org http://www.optimizationservices.org/schemas/2.0/OSoL.xsd");

    osolDocument.InsertFirstChild(osolNode);

    auto optimizationNode = osolDocument.NewElement("optimization");

    auto solverOptionsNode = osolDocument.NewElement("solverOptions");
    int numberOfIncludedOptions = 0;

    solverOptionsNode->SetAttribute("numberOfSolverOptions", numberOfIncludedOptions);

    for(SettingsIter iterator = settings.begin(); iterator != settings.end(); iterator++)
    {
        auto p = iterator->first;

        if(settingIsPrivate[p])
            continue; // Do not include an internal setting

        std::stringstream type;

        switch(settingTypes[p])
        {
        case E_SettingType::String:
            type << "string";
            break;
        case E_SettingType::Integer:
            type << "integer";
            break;
        case E_SettingType::Boolean:
            type << "integer";
            break;
        case E_SettingType::Enum:
            type << "integer";
            break;
        case E_SettingType::Double:
            type << "double";
        }

        std::stringstream desc;

        if(settingEnums[p] == true)
        {
            desc << settingDescriptions[p] << ": " << getEnumDescriptionList(p.second, p.first);
        }
        else
        {
            desc << settingDescriptions[p] << ". ";
        }

        auto solverOptionNode = osolDocument.NewElement("solverOption");
        solverOptionNode->SetAttribute("name", p.second.c_str());
        solverOptionNode->SetAttribute("value", iterator->second.c_str());
        solverOptionNode->SetAttribute("solver", std::string("SHOT").c_str());
        solverOptionNode->SetAttribute("category", p.first.c_str());
        solverOptionNode->SetAttribute("value", type.str().c_str());
        solverOptionNode->SetAttribute("description", desc.str().c_str());
        solverOptionsNode->InsertEndChild(solverOptionNode);
        solverOptionNode++;

        output->outputDebug(" Setting <" + p.first + "," + p.second + "> converted.");

        type.clear();
        desc.clear();
    }

    optimizationNode->InsertEndChild(solverOptionsNode);
    osolNode->InsertEndChild(optimizationNode);

    XMLPrinter printer;
    osolDocument.Print(&printer);

    return (printer.CStr());
}

bool operator<(const PairString& lhs, const PairString& rhs) { return lhs.first < rhs.first; }

std::string Settings::getSettingsAsString(bool hideUnchanged = false, bool hideDescriptions = false)
{
    std::stringstream ss;

    for(SettingsIter iterator = settings.begin(); iterator != settings.end(); iterator++)
    {
        auto p = iterator->first;

        if(settingIsPrivate[p])
            continue; // Do not include an internal setting

        if(hideUnchanged && settingIsDefaultValue[p])
            continue; // Hide setting with default value

        if(!hideDescriptions)
        {
            std::stringstream desc;

            if(settingEnums[p] == true)
            {
                desc << settingDescriptions[p] << ": " << getEnumDescriptionList(p.second, p.first);
            }
            else
            {
                desc << settingDescriptions[p] << ". ";
            }

            if(((int)desc.tellp()) != 0)
            {
                ss << fmt::format("* {}\n", desc.str());
            }
        }

        ss << fmt::format("{}.{} = {}\n\n", p.first, p.second, iterator->second);
    }

    return (ss.str());
}

void Settings::readSettingsFromOSoL(std::string osol)
{
    using namespace tinyxml2;

    output->outputTrace("Starting conversion of settings from OSoL.");

    XMLDocument osolDocument;

    auto result = osolDocument.Parse(osol.c_str());

    if(result != XML_SUCCESS)
    {
        output->outputError("Could not parse options in OSoL-format.", std::to_string(result));
        return;
    }

    auto osolNode
        = osolDocument.FirstChildElement("osol")->FirstChildElement("optimization")->FirstChildElement("solverOptions");

    if(osolNode == nullptr)
    {
        output->outputError("No solver options specified in OSoL-file.");
        return;
    }

    for(auto N = osolNode->FirstChildElement("solverOption"); N != NULL; N = N->NextSiblingElement("solverOption"))
    {
        try
        {
            std::string solver = N->Attribute("solver");

            if(solver != "SHOT" && solver != "shot")
                continue;

            std::string name = N->Attribute("name");

            std::string value; // For some reason, value can be empty
            if(N->Attribute("value") != nullptr)
            {
                value = N->Attribute("value");
            }

            std::string category = N->Attribute("category");

            PairString key = make_pair(category, name);
            settingsIterator = settings.find(key);

            if(settingsIterator == settings.end())
            {
                output->outputError(
                    "Cannot update setting <" + category + "," + name + "> since it has not been defined.");

                throw SettingKeyNotFoundException(name, category);
            }

            std::string::size_type convertedChars = value.length();

            switch(settingTypes[key])
            {
            case E_SettingType::String:
                updateSetting(name, category, value);
                break;
            case E_SettingType::Enum:
            case E_SettingType::Integer:
                updateSetting(name, category, std::stoi(value, &convertedChars));
                break;
            case E_SettingType::Boolean:
            {
                bool convertedValue = (value != "0");
                updateSetting(name, category, convertedValue);
                break;
            }
            case E_SettingType::Double:
                updateSetting(name, category, std::stod(value));
                break;
            default:
                break;
            }

            if(convertedChars != value.length())
                output->outputError(
                    "Cannot update setting <" + name + "," + category + "> since it is of the wrong type.");
        }
        catch(std::exception& e)
        {
            output->outputError("Error when reading OSoL line " + std::to_string(N->GetLineNum()));
        }
    }
}

void Settings::readSettingsFromString(std::string options)
{
    output->outputTrace("Starting conversion of settings from GAMS options format.");

    std::istringstream f(options);
    std::string line;

    while(std::getline(f, line))
    {
        // Ignore empty lines and comments (starting with an asterisk)
        if(line == "" || line.at(0) == '*')
            continue;

        int equalitySignIndex = line.find('=');

        std::string key = line.substr(0, equalitySignIndex);
        std::string value = line.substr(equalitySignIndex + 1, line.size());

        int dotIndex = line.find('.');
        std::string category = key.substr(0, dotIndex);
        std::string name = key.substr(dotIndex + 1, line.size());

        if(key == "" || value == "")
        {
            output->outputError("Error when reading line \"" + line + "\" in the options file; ignoring the option.");
            continue;
        }

        category = UtilityFunctions::trim(category);
        name = UtilityFunctions::trim(name);
        value = UtilityFunctions::trim(value);

        PairString keyCategory = make_pair(category, name);
        settingsIterator = settings.find(keyCategory);

        if(settingsIterator == settings.end())
        {
            output->outputError("Cannot update setting <" + name + "," + category + "> since it has not been defined.");

            throw SettingKeyNotFoundException(name, category);
        }

        std::string::size_type convertedChars = value.length();

        switch(settingTypes[keyCategory])
        {
        case E_SettingType::String:
            updateSetting(name, category, value);
            break;
        case E_SettingType::Enum:
        case E_SettingType::Integer:
            updateSetting(name, category, std::stoi(value, &convertedChars));
            break;
        case E_SettingType::Boolean:
        {
            bool convertedValue = (value != "0");
            updateSetting(name, category, convertedValue);
            break;
        }
        case E_SettingType::Double:
            updateSetting(name, category, std::stod(value));
            break;
        default:
            break;
        }

        if(convertedChars != value.length())
            output->outputError("Cannot update setting <" + name + "," + category + "> since it is of the wrong type.");
    }
}
} // namespace SHOT