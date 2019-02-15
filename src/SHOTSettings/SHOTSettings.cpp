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
    std::string oldvalue = _settings[key];

    if(oldvalue == value)
    {
        output->outputTrace(
            "Setting " + key.first + "." + key.second + " not updated. Same value " + oldvalue + " given.");
        return;
    }
    else
    {
        _settings[key] = value;
        _isDefault[key] = false;

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
    PairString key = make_pair(name, category);
    _settings[key] = value;
    _settingsDesc[key] = description;
    _settingsType[key] = E_SettingsType::String;
    std::string test = name;
    _isPrivate[key] = isPrivate;
    _isDefault[key] = true;

    output->outputTrace("Setting <" + name + "," + category + "> = " + value + " created.");
}

void Settings::updateSetting(std::string name, std::string category, std::string value)
{
    PairString key = make_pair(name, category);
    _settingsIter = _settings.find(key);

    if(_settingsIter == _settings.end())
    {
        output->outputError("Cannot update setting <" + name + "," + category + "> since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(_settingsType[key] != E_SettingsType::String)
    {
        output->outputError(
            "Cannot update setting <" + name + "," + category + "> since it is of the wrong type. (Expected string).");

        throw SettingSetWrongTypeException(name, category);
    }

    updateSettingBase(key, value);
}

std::string Settings::getStringSetting(std::string name, std::string category)
{
    PairString key = make_pair(name, category);
    _settingsIter = _settings.find(key);

    if(_settingsIter == _settings.end())
    {
        output->outputError("Cannot get setting <" + name + "," + category + "> since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(_settingsType[key] != E_SettingsType::String)
    {
        output->outputError(
            "Cannot get value of setting <" + name + "," + category + "> as string: Wrong type requested!");

        throw SettingGetWrongTypeException(name, category);
    }

    return _settings[key];
}

// Integer settings ==============================================================

void Settings::createSetting(std::string name, std::string category, int value, std::string description, double minVal,
    double maxVal, bool isPrivate)
{
    PairString key = make_pair(name, category);
    _settings[key] = std::to_string(value);
    _settingsDesc[key] = description;
    _settingsType[key] = E_SettingsType::Integer;
    _settingsBounds[key] = std::make_pair(minVal, maxVal);
    _isPrivate[key] = isPrivate;
    _isDefault[key] = true;

    output->outputTrace("Setting <" + name + "," + category + "> = " + std::to_string(value) + " created.");
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
    PairString key = make_pair(name, category);
    _settingsIter = _settings.find(key);

    if(_settingsIter == _settings.end())
    {
        output->outputError("Cannot update setting <" + name + "," + category + "> since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(_settingsType[key] != E_SettingsType::Integer)
    {
        output->outputError("Cannot set value of setting <" + name + "," + category + "> as integer: Wrong type!");

        throw SettingSetWrongTypeException(name, category);
    }

    if(_settingsBounds[key].first > value || _settingsBounds[key].second < value)
    {
        output->outputError("Cannot update setting <" + name + "," + category + ">: Not in interval ["
            + std::to_string(_settingsBounds[key].first) + "," + std::to_string(_settingsBounds[key].second) + "].");

        throw SettingOutsideBoundsException(
            name, category, (double)value, _settingsBounds[key].first, _settingsBounds[key].second);
    }

    updateSettingBase(key, std::to_string(value));
}

int Settings::getIntSetting(std::string name, std::string category)
{
    PairString key = make_pair(name, category);
    _settingsIter = _settings.find(key);

    if(_settingsIter == _settings.end())
    {
        output->outputError("Cannot get setting <" + name + "," + category + "> since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(_settingsType[key] != E_SettingsType::Integer)
    {
        output->outputError(
            "Cannot get value of setting <" + name + "," + category + "> as integer: Wrong type requested!");

        throw SettingGetWrongTypeException(name, category);
    }

    int intval = std::stoi(_settings[key]);
    return intval;
}

// Boolean settings ==============================================================

void Settings::createSetting(
    std::string name, std::string category, bool value, std::string description, bool isPrivate)
{
    PairString key = make_pair(name, category);
    _settings[key] = std::to_string(value);
    _settingsDesc[key] = description;
    _settingsType[key] = E_SettingsType::Boolean;
    _isPrivate[key] = isPrivate;
    _isDefault[key] = true;

    output->outputTrace("Setting <" + name + "," + category + "> = " + std::to_string(value) + " created.");
}

void Settings::createSetting(std::string name, std::string category, bool value, std::string description)
{
    Settings::createSetting(name, category, value, description, false);
}

void Settings::updateSetting(std::string name, std::string category, bool value)
{
    PairString key = make_pair(name, category);
    _settingsIter = _settings.find(key);

    if(_settingsIter == _settings.end())
    {
        output->outputError("Cannot update setting <" + name + "," + category + "> since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(_settingsType[key] != E_SettingsType::Boolean)
    {
        output->outputError("Cannot set value of setting <" + name + "," + category + "> as bool: Wrong type!");

        throw SettingSetWrongTypeException(name, category);
    }

    std::string newvalue = std::to_string(value);

    updateSettingBase(key, newvalue);
}

bool Settings::getBoolSetting(std::string name, std::string category)
{
    PairString key = make_pair(name, category);
    _settingsIter = _settings.find(key);

    if(_settingsIter == _settings.end())
    {
        output->outputError("Cannot get setting <" + name + "," + category + "> since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(_settingsType[key] != E_SettingsType::Boolean)
    {
        output->outputError(
            "Cannot get value of setting <" + name + "," + category + "> as boolean: Wrong type requested!");

        throw SettingGetWrongTypeException(name, category);
    }

    bool boolval = (_settings[key] != "0");
    return boolval;
}

// Enum settings ==================================================================

void Settings::createSetting(std::string name, std::string category, int value, std::string description,
    VectorString enumDescriptions, bool isPrivate)
{
    createSetting(name, category, value, description, SHOT_DBL_MIN, SHOT_DBL_MAX, isPrivate);

    for(int i = 0; i < enumDescriptions.size(); i++)
    {
        _enumDescription[make_tuple(name, category, i)] = enumDescriptions.at(i);

        output->outputTrace(" Enum value " + std::to_string(i) + ": " + enumDescriptions.at(i));
    }

    _settingsEnum[make_pair(name, category)] = true;
}

void Settings::createSetting(
    std::string name, std::string category, int value, std::string description, VectorString enumDescriptions)
{
    Settings::createSetting(name, category, value, description, enumDescriptions, false);
}

std::string Settings::getEnumDescriptionList(std::string name, std::string category)
{
    std::stringstream desc;

    for(EnumDescriptionIter iterator = _enumDescription.begin(); iterator != _enumDescription.end(); iterator++)
    {
        std::tuple<std::string, std::string, int> t = iterator->first;

        if(name == std::get<0>(t) && category == std::get<1>(t))
        {
            desc << std::get<2>(t) << ": " << iterator->second << ". ";
        }
    }

    return desc.str();
}

std::string Settings::getEnumDescription(std::string name, std::string category)
{
    PairString key = make_pair(name, category);
    _settingsIter = _settings.find(key);

    if(_settingsIter == _settings.end())
    {
        output->outputError("Cannot get setting <" + name + "," + category + "> since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(_settingsType[key] != E_SettingsType::Integer)
    {
        output->outputError(
            "Cannot get value of setting <" + name + "," + category + "> as integer: Wrong type requested!");

        throw SettingGetWrongTypeException(name, category);
    }

    int intval = std::stoi(_settings[key]);
    std::tuple<std::string, std::string, int> tpl = make_tuple(name, category, intval);

    return _enumDescription[tpl];
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
    PairString key = make_pair(name, category);
    _settings[key] = std::to_string(value);
    _settingsDesc[key] = description;
    _settingsType[key] = E_SettingsType::Double;
    _settingsBounds[key] = std::make_pair(minVal, maxVal);
    _isPrivate[key] = isPrivate;
    _isDefault[key] = true;

    output->outputTrace("Setting <" + name + "," + category + "> = " + std::to_string(value) + " created.");
}

void Settings::updateSetting(std::string name, std::string category, double value)
{
    PairString key = make_pair(name, category);
    _settingsIter = _settings.find(key);

    if(_settingsIter == _settings.end())
    {
        output->outputError("Cannot update setting <" + name + "," + category + "> since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(_settingsType[key] != E_SettingsType::Double)
    {
        output->outputError("Cannot set value of setting <" + name + "," + category + "> as double: Wrong type!");

        throw SettingSetWrongTypeException(name, category);
    }

    if(_settingsBounds[key].first > value || _settingsBounds[key].second < value)
    {
        output->outputError("Cannot update setting <" + name + "," + category + ">: Not in interval ["
            + std::to_string(_settingsBounds[key].first) + "," + std::to_string(_settingsBounds[key].second) + "].");

        throw SettingOutsideBoundsException(
            name, category, value, _settingsBounds[key].first, _settingsBounds[key].second);
    }

    std::string newvalue = std::to_string(value);

    updateSettingBase(key, newvalue);
}

double Settings::getDoubleSetting(std::string name, std::string category)
{
    PairString key = make_pair(name, category);
    _settingsIter = _settings.find(key);

    if(_settingsIter == _settings.end())
    {
        output->outputError("Cannot get setting <" + name + "," + category + "> since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    if(_settingsType[key] != E_SettingsType::Double)
    {
        output->outputError(
            "Cannot get value of setting <" + name + "," + category + "> as double: Wrong type requested!");

        throw SettingGetWrongTypeException(name, category);
    }

    double intval = std::stod(_settings[key]);
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

    for(SettingsIter iterator = _settings.begin(); iterator != _settings.end(); iterator++)
    {
        auto p = iterator->first;

        if(_isPrivate[p])
            continue; // Do not include an internal setting

        std::stringstream type;

        switch(_settingsType[p])
        {
        case E_SettingsType::String:
            type << "string";
            break;
        case E_SettingsType::Integer:
            type << "integer";
            break;
        case E_SettingsType::Boolean:
            type << "integer";
            break;
        case E_SettingsType::Enum:
            type << "integer";
            break;
        case E_SettingsType::Double:
            type << "double";
        }

        std::stringstream desc;

        if(_settingsEnum[p] == true)
        {
            desc << _settingsDesc[p] << ": " << getEnumDescriptionList(p.first, p.second);
        }
        else
        {
            desc << _settingsDesc[p] << ". ";
        }

        auto solverOptionNode = osolDocument.NewElement("solverOption");
        solverOptionNode->SetAttribute("name", p.first.c_str());
        solverOptionNode->SetAttribute("value", iterator->second.c_str());
        solverOptionNode->SetAttribute("solver", std::string("SHOT").c_str());
        solverOptionNode->SetAttribute("category", p.second.c_str());
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

std::string Settings::getSettingsAsString(bool hideUnchanged = false, bool hideDescriptions = false)
{
    std::stringstream ss;

    for(SettingsIter iterator = _settings.begin(); iterator != _settings.end(); iterator++)
    {
        auto p = iterator->first;

        if(_isPrivate[p])
            continue; // Do not include an internal setting

        if(hideUnchanged && _isDefault[p])
            continue;

        if(!hideDescriptions)
        {
            std::stringstream desc;

            if(_settingsEnum[p] == true)
            {
                desc << _settingsDesc[p] << ": " << getEnumDescriptionList(p.first, p.second);
            }
            else
            {
                desc << _settingsDesc[p] << ". ";
            }

            if(((int)desc.tellp()) != 0)
            {
                ss << fmt::format("* {}", desc.str());
            }
        }

        ss << fmt::format("{}.{} = {}", p.second, p.first, iterator->second);
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

            PairString key = make_pair(name, category);
            _settingsIter = _settings.find(key);

            if(_settingsIter == _settings.end())
            {
                output->outputError(
                    "Cannot update setting <" + name + "," + category + "> since it has not been defined.");

                throw SettingKeyNotFoundException(name, category);
            }

            std::string::size_type convertedChars = value.length();

            switch(_settingsType[key])
            {
            case E_SettingsType::String:
                updateSetting(name, category, value);
                break;
            case E_SettingsType::Enum:
            case E_SettingsType::Integer:
                updateSetting(name, category, std::stoi(value, &convertedChars));
                break;
            case E_SettingsType::Boolean:
            {
                bool convertedValue = (value != "0");
                updateSetting(name, category, convertedValue);
                break;
            }
            case E_SettingsType::Double:
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
        if(line == "" || boost::algorithm::starts_with(line, "*"))
            continue;

        VectorString nameCategoryPair;
        VectorString keyValuePair;
        boost::split(keyValuePair, line, boost::is_any_of("="));

        int dotindex = line.find('.');
        std::string category = keyValuePair.at(0).substr(0, dotindex);
        std::string name = keyValuePair.at(0).substr(dotindex + 1, line.size());

        if(keyValuePair.size() != 2)
        {
            output->outputError("Error when reading line \"" + line + "\" in the options file; ignoring the option.");

            continue;
        }

        std::string value = keyValuePair.at(1);

        boost::trim(category);
        boost::trim(name);
        boost::trim(value);

        PairString key = make_pair(name, category);
        _settingsIter = _settings.find(key);

        if(_settingsIter == _settings.end())
        {
            output->outputError("Cannot update setting <" + name + "," + category + "> since it has not been defined.");

            throw SettingKeyNotFoundException(name, category);
        }

        std::string::size_type convertedChars = value.length();

        switch(_settingsType[key])
        {
        case E_SettingsType::String:
            updateSetting(name, category, value);
            break;
        case E_SettingsType::Enum:
        case E_SettingsType::Integer:
            updateSetting(name, category, std::stoi(value, &convertedChars));
            break;
        case E_SettingsType::Boolean:
        {
            bool convertedValue = (value != "0");
            updateSetting(name, category, convertedValue);
            break;
        }
        case E_SettingsType::Double:
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