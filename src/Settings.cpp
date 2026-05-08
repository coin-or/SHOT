/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Settings.h"
#include "Utilities.h"

#include "tinyxml2.h"

#include <algorithm>
#include <set>
#include <sstream>

namespace SHOT
{
Settings::Settings(OutputPtr outputPtr) : output(outputPtr) { }

Settings::~Settings() { }

template <typename T>
void Settings::createBaseSetting(
    std::string name, std::string category, T value, std::string description, bool isPrivate)
{
    // Check that setting is of the correct type
    using value_type [[maybe_unused]] = typename std::enable_if<std::is_same<std::string, T>::value
            || std::is_same<double, T>::value || std::is_same<int, T>::value || std::is_same<bool, T>::value,
        T>::type;

    PairString key = make_pair(category, name);

    std::string tempValue;

    if constexpr(std::is_same_v<T, std::string>)
    {
        stringSettings[key] = value;
        settingTypes[key] = E_SettingType::String;
        tempValue = Utilities::trim(value);
        output->outputTrace(" String setting " + category + "." + name + " = " + tempValue + " created.");
    }
    else if constexpr(std::is_same_v<T, int>)
    {
        integerSettings[key] = value;
        settingTypes[key] = E_SettingType::Integer;
        tempValue = std::to_string(value);
        output->outputTrace(" Integer setting " + category + "." + name + " = " + tempValue + " created.");
    }
    else if constexpr(std::is_same_v<T, double>)
    {
        doubleSettings[key] = value;
        settingTypes[key] = E_SettingType::Double;
        tempValue = std::to_string(value);
        output->outputTrace(" Double setting " + category + "." + name + " = " + tempValue + " created.");
    }
    else if constexpr(std::is_same_v<T, bool>)
    {
        booleanSettings[key] = value;
        settingTypes[key] = E_SettingType::Boolean;
        tempValue = std::to_string(value);
        output->outputTrace(" Boolean " + category + "." + name + " = " + tempValue + " created.");
    }

    settingDescriptions[key] = description;
    settingIsPrivate[key] = isPrivate;
    settingActivePriority[key] = static_cast<int>(E_SettingPriority::Default);

    // Record initial value in history at Default priority (0)
    if constexpr(std::is_same_v<T, std::string>)
        stringSettingHistory[key][0] = value;
    else if constexpr(std::is_same_v<T, int>)
        integerSettingHistory[key][0] = value;
    else if constexpr(std::is_same_v<T, double>)
        doubleSettingHistory[key][0] = value;
    else if constexpr(std::is_same_v<T, bool>)
        booleanSettingHistory[key][0] = value;
}

template void Settings::updateSetting(
    std::string name, std::string category, std::string value, E_SettingPriority priority);
template void Settings::updateSetting(
    std::string name, std::string category, int value, E_SettingPriority priority);
template void Settings::updateSetting(
    std::string name, std::string category, double value, E_SettingPriority priority);
template void Settings::updateSetting(
    std::string name, std::string category, bool value, E_SettingPriority priority);

static std::string priorityLabel(E_SettingPriority p)
{
    switch(p)
    {
    case E_SettingPriority::RecommendedInternal: return "recommended";
    case E_SettingPriority::OptionsFile:         return "options file";
    case E_SettingPriority::UserAPI:             return "user";
    case E_SettingPriority::SolverInternal:      return "solver";
    case E_SettingPriority::SolverCompatibility: return "solver (compatibility)";
    default:                                     return "default";
    }
}

template <typename T>
void Settings::updateSetting(std::string name, std::string category, T value, E_SettingPriority priority)
{
    // Check that setting is of the correct type
    using value_type [[maybe_unused]] = typename std::enable_if<std::is_same<std::string, T>::value
            || std::is_same<double, T>::value || std::is_same<int, T>::value || std::is_same<bool, T>::value,
        T>::type;

    PairString key = make_pair(category, name);

    typename std::map<PairString, T>::iterator oldValue;
    typename std::map<PairString, T>::iterator end;

    if constexpr(std::is_same_v<T, std::string>)
    {
        oldValue = stringSettings.find(key);
        end = stringSettings.end();
    }
    else if constexpr(std::is_same_v<T, int>)
    {
        if(settingBounds[key].first > value || settingBounds[key].second < value)
        {
            output->outputError(" Cannot update setting " + category + "." + name + ": Not in interval ["
                + std::to_string(settingBounds[key].first) + "," + std::to_string(settingBounds[key].second) + "].");

            throw SettingOutsideBoundsException(
                name, category, (double)value, settingBounds[key].first, settingBounds[key].second);
        }

        oldValue = integerSettings.find(key);
        end = integerSettings.end();
    }
    else if constexpr(std::is_same_v<T, double>)
    {
        if(settingBounds[key].first > value || settingBounds[key].second < value)
        {
            output->outputError(" Cannot update setting " + category + "." + name + ": Not in interval ["
                + std::to_string(settingBounds[key].first) + "," + std::to_string(settingBounds[key].second) + "].");

            throw SettingOutsideBoundsException(
                name, category, (double)value, settingBounds[key].first, settingBounds[key].second);
        }

        oldValue = doubleSettings.find(key);
        end = doubleSettings.end();
    }
    else if constexpr(std::is_same_v<T, bool>)
    {
        oldValue = booleanSettings.find(key);
        end = booleanSettings.end();
    }

    if(oldValue == end)
    {
        output->outputError("Cannot update setting " + category + "." + name + " since it has not been defined.");

        throw SettingKeyNotFoundException(name, category);
    }

    int prio = static_cast<int>(priority);

    // Always record the write in per-priority history
    if constexpr(std::is_same_v<T, std::string>)
        stringSettingHistory[key][prio] = Utilities::trim(value);
    else if constexpr(std::is_same_v<T, int>)
        integerSettingHistory[key][prio] = value;
    else if constexpr(std::is_same_v<T, double>)
        doubleSettingHistory[key][prio] = value;
    else if constexpr(std::is_same_v<T, bool>)
        booleanSettingHistory[key][prio] = value;

    // Only promote to active if this priority level wins
    if(prio < settingActivePriority[key])
    {
        output->outputTrace(" Setting " + key.first + "." + key.second + " stored at priority "
            + std::to_string(prio) + " but not activated (active priority is "
            + std::to_string(settingActivePriority[key]) + ").");
        return;
    }

    // Skip active update when the value and priority are identical to the current active state
    bool valueUnchanged;
    if constexpr(std::is_same_v<T, std::string>)
        valueUnchanged = (Utilities::trim(oldValue->second) == Utilities::trim(value));
    else
        valueUnchanged = (oldValue->second == value);

    if(valueUnchanged && prio == settingActivePriority[key])
    {
        output->outputTrace(
            " Setting " + key.first + "." + key.second + " not updated since the same value was given.");
        return;
    }

    if constexpr(std::is_same_v<T, std::string>)
    {
        stringSettings[key] = Utilities::trim(value);

        output->outputTrace(" Setting " + key.first + "." + key.second + " updated. New value = " + value + ".");
    }
    else if constexpr(std::is_same_v<T, int>)
    {
        integerSettings[key] = value;

        output->outputTrace(
            " Setting " + key.first + "." + key.second + " updated. New value = " + std::to_string(value) + ".");
    }
    else if constexpr(std::is_same_v<T, double>)
    {
        doubleSettings[key] = value;

        output->outputTrace(
            " Setting " + key.first + "." + key.second + " updated. New value = " + std::to_string(value) + ".");
    }
    else if constexpr(std::is_same_v<T, bool>)
    {
        booleanSettings[key] = value;

        output->outputTrace(
            " Setting " + key.first + "." + key.second + " updated. New value = " + std::to_string(value) + ".");
    }

    settingActivePriority[key] = prio;
}

// String settings ===============================================================

void Settings::createSetting(
    std::string name, std::string category, std::string value, std::string description, bool isPrivate)
{
    createBaseSetting<std::string>(name, category, value, description, isPrivate);
}

// Integer settings ==============================================================

void Settings::createSetting(std::string name, std::string category, int value, std::string description, double minVal,
    double maxVal, bool isPrivate)
{
    createBaseSetting<int>(name, category, value, description, isPrivate);
    settingBounds[make_pair(category, name)] = std::make_pair(minVal, maxVal);
}

// Double settings ===============================================================

void Settings::createSetting(std::string name, std::string category, double value, std::string description,
    double minVal, double maxVal, bool isPrivate)
{
    createBaseSetting<double>(name, category, value, description, isPrivate);
    settingBounds[make_pair(category, name)] = std::make_pair(minVal, maxVal);
}

// Boolean settings ==============================================================

void Settings::createSetting(
    std::string name, std::string category, bool value, std::string description, bool isPrivate)
{
    createBaseSetting<bool>(name, category, value, description, isPrivate);
}

// Enum settings ==================================================================

void Settings::createSetting(std::string name, std::string category, int value, std::string description,
    VectorString enumDesc, int startValue, bool isPrivate)
{
    createBaseSetting<int>(name, category, value, description, isPrivate);
    settingBounds[make_pair(category, name)]
        = std::make_pair((double)startValue, (double)(startValue + enumDesc.size() - 1));

    size_t counter = 0;

    for(int i = startValue; i < (int)(startValue + enumDesc.size()); i++)
    {
        enumDescriptions[make_tuple(category, name, i)] = enumDesc.at(counter);
        output->outputTrace(" Enum value " + std::to_string(i) + ": " + enumDesc.at(counter));
        counter++;
    }

    settingEnums[make_pair(category, name)] = true;
}

std::string Settings::getEnumDescriptionList(std::string name, std::string category)
{
    std::stringstream desc;

    for(auto& E : enumDescriptions)
    {
        if(name == std::get<1>(E.first) && category == std::get<0>(E.first))
        {
            desc << std::get<2>(E.first) << ": " << E.second << ". ";
        }
    }

    return desc.str();
}

std::string Settings::getEnumDescriptionListMarkup(std::string name, std::string category)
{
    std::stringstream desc;

    for(auto& E : enumDescriptions)
    {
        if(name == std::get<1>(E.first) && category == std::get<0>(E.first))
        {
            desc << std::get<2>(E.first) << ": " << E.second << " ";
        }
    }

    return desc.str();
}

std::vector<std::pair<int, std::string>> Settings::getEnumDescription(std::string name, std::string category)
{
    std::vector<std::pair<int, std::string>> r;

    for(auto& E : enumDescriptions)
    {
        if(name == std::get<1>(E.first) && category == std::get<0>(E.first))
        {
            r.push_back(std::pair<int, std::string>(std::get<2>(E.first), E.second));
        }
    }

    return r;
}

// General methods ================================================================

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

    for(auto& T : settingTypes)
    {
        auto key = T.first;
        std::string name = T.first.second;
        std::string category = T.first.first;

        if(settingIsPrivate[key])
            continue; // Do not include an internal setting

        std::stringstream type;
        std::string value;

        switch(settingTypes[key])
        {
        case E_SettingType::String:
            type << "string";
            value = getSetting<std::string>(name, category);
            break;
        case E_SettingType::Integer:
            type << "integer";
            value = fmt::format("{}", getSetting<int>(name, category));
            break;
        case E_SettingType::Boolean:
            type << "boolean";
            value = fmt::format("{}", getSetting<bool>(name, category));
            break;
        case E_SettingType::Enum:
            type << "integer";
            value = fmt::format("{}", getSetting<int>(name, category));
            break;
        case E_SettingType::Double:
            type << "double";
            value = fmt::format("{}", getSetting<double>(name, category));
        }

        std::stringstream desc;

        if(settingEnums[key] == true)
        {
            desc << settingDescriptions[key] << ": " << getEnumDescriptionList(name, category);
        }
        else
        {
            desc << settingDescriptions[key] << ". ";
        }

        auto solverOptionNode = osolDocument.NewElement("solverOption");
        solverOptionNode->SetAttribute("name", name.c_str());
        solverOptionNode->SetAttribute("value", value.c_str());
        solverOptionNode->SetAttribute("solver", std::string("SHOT").c_str());
        solverOptionNode->SetAttribute("category", category.c_str());
        solverOptionNode->SetAttribute("type", type.str().c_str());
        solverOptionNode->SetAttribute("description", desc.str().c_str());
        solverOptionsNode->InsertEndChild(solverOptionNode);
        solverOptionNode++;

        output->outputDebug(" Setting <" + category + "," + name + "> converted.");

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

    std::string currentCategory = "";
    std::string currentSubCategory = "";

    const std::string divider = "**************************************************************************************"
                                "****************************************************";

    for(auto& T : settingTypes)
    {
        auto key = T.first;
        std::string name = T.first.second;
        std::string category = T.first.first;

        if(settingIsPrivate[key])
            continue; // Do not include an internal setting

        if(hideUnchanged && settingActivePriority[key] == 0)
            continue; // Hide setting with default value

        if(!hideDescriptions)
        {
            std::string name = T.first.second;
            std::string category = T.first.first;
            std::string fullname = fmt::format("{}.{}", category, name);
            std::string subCategory = "";

            size_t firstPeriod = name.find('.');

            if(firstPeriod != std::string::npos)
                subCategory = name.substr(0, firstPeriod);
            else
                subCategory = name;

            std::string description;
            std::string validValues;
            std::string defaultValue;

            if(category != currentCategory)
            {
                // This is a first level group

                auto [header, description] = settingGroupDescriptions[std::make_pair(category, "")];

                ss << '\n' << '\n' << divider << '\n';
                ss << divider << '\n';
                ss << fmt::format("* {}\n", header);

                if(description != "")
                    ss << "* " << description << "\n";

                ss << divider << '\n' << divider << '\n';

                currentCategory = category;
                currentSubCategory = "something";
            }

            if(subCategory != currentSubCategory
                && (settingGroupDescriptions.find(std::make_pair(category, subCategory))
                    != settingGroupDescriptions.end()))
            {
                // This is a second level group

                auto [header, description] = settingGroupDescriptions[std::make_pair(category, subCategory)];

                ss << '\n' << '\n' << divider << '\n';
                ss << fmt::format("* {}\n", header);

                if(Utilities::trim(description) != "")
                    ss << "* " << description << "\n";

                ss << divider << '\n';

                currentSubCategory = subCategory;
            }

            std::stringstream desc;

            if(settingEnums[key] == true)
            {
                desc << settingDescriptions[key] << ": " << getEnumDescriptionList(name, category);
            }
            else
            {
                desc << settingDescriptions[key] << ". ";
            }

            if(((int)desc.tellp()) != 0)
            {
                ss << fmt::format("\n* {}\n", desc.str());
            }
        }

        switch(T.second)
        {
        case(E_SettingType::String):
            ss << fmt::format("{}.{} = {}\n", category, name, getSetting<std::string>(name, category));
            break;

        case(E_SettingType::Double):
            ss << fmt::format("{}.{} = {}\n", category, name, getSetting<double>(name, category));
            break;

        case(E_SettingType::Integer):
            ss << fmt::format("{}.{} = {}\n", category, name, getSetting<int>(name, category));
            break;

        case(E_SettingType::Enum):
            ss << fmt::format("{}.{} = {}\n", category, name, getSetting<int>(name, category));
            break;

        case(E_SettingType::Boolean):
            ss << fmt::format("{}.{} = {}\n", category, name, getSetting<bool>(name, category));
            break;
        default:
            break;
        }
    }

    return (ss.str());
}

std::string Settings::getSettingsAsMarkup()
{
    std::stringstream ss;

    std::string currentCategory = "";
    std::string currentSubCategory = "";

    for(auto& T : settingTypes)
    {
        auto key = T.first;

        if(settingIsPrivate[key])
            continue; // Do not include an internal setting

        std::string name = T.first.second;
        std::string category = T.first.first;
        std::string fullname = fmt::format("{}.{}", category, name);
        std::string subCategory = "";

        size_t firstPeriod = name.find('.');

        if(firstPeriod != std::string::npos)
            subCategory = name.substr(0, firstPeriod);
        else
            subCategory = name;

        std::string description;
        std::string validValues;
        std::string defaultValue;

        bool printTableHeader = false;

        if(category != currentCategory)
        {
            // This is a first level group

            auto [header, description] = settingGroupDescriptions[std::make_pair(category, "")];

            ss << '\n' << fmt::format("# {}\n", header) << '\n';

            if(description != "")
                ss << description << "\n\n";

            currentCategory = category;
            currentSubCategory = "something";
            printTableHeader = true;
        }

        if(subCategory != currentSubCategory
            && (settingGroupDescriptions.find(std::make_pair(category, subCategory)) != settingGroupDescriptions.end()))
        {
            // This is a second level group

            auto [header, description] = settingGroupDescriptions[std::make_pair(category, subCategory)];

            ss << '\n' << fmt::format("## {}\n", header) << '\n';

            if(Utilities::trim(description) != "")
                ss << description << "\n\n";

            currentSubCategory = subCategory;
            printTableHeader = true;
        }

        if(printTableHeader)
        {
            ss << fmt::format("|Name and description|Valid values|Default value|\n");
            ss << fmt::format("|-|:-:|:-:|\n");
        }

        if(settingEnums[key] == true)
        {
            description = fmt::format(
                "**{}**<br>{}<br>{}", fullname, settingDescriptions[key], getEnumDescriptionListMarkup(name, category));
        }
        else
        {
            description = fmt::format("**{}**<br>{}", fullname, settingDescriptions[key]);
        }

        PairDouble bounds;

        switch(T.second)
        {
        case(E_SettingType::String):
            validValues = fmt::format("string");
            defaultValue = getSetting<std::string>(name, category);
            break;

        case(E_SettingType::Double):
            bounds = settingBounds[key];
            validValues = fmt::format("[{},{}]", Utilities::toStringFormat(bounds.first, "{}", true, "∞"),
                Utilities::toStringFormat(bounds.second, "{}", true, "∞"));
            defaultValue = fmt::format("{}", getSetting<double>(name, category));
            break;

        case(E_SettingType::Integer):
            bounds = settingBounds[key];

            if(std::round(bounds.second) == std::round(bounds.first) + 1)
            {
                validValues = fmt::format("{{{},{}}}", (int)bounds.first, (int)bounds.second);
            }
            else if(std::round(bounds.second) == SHOT_INT_MAX)
            {
                validValues = fmt::format("{{{},...,∞}}", (int)bounds.first);
            }
            else
            {
                validValues = fmt::format("{{{},...,{}}}", (int)bounds.first, (int)bounds.second);
            }

            defaultValue = fmt::format("{}", getSetting<int>(name, category));
            break;

        case(E_SettingType::Enum):
            bounds = settingBounds[key];

            if(std::round(bounds.second) == std::round(bounds.first) + 1)
            {
                validValues = fmt::format("{{{},{}}}", bounds.first, bounds.second);
            }
            else if(std::round(bounds.second) == SHOT_INT_MAX)
            {
                validValues = fmt::format("{{{},...,∞}}", bounds.first);
            }
            else
            {
                validValues = fmt::format("{{{},...,{}}}", bounds.first, bounds.second);
            }

            defaultValue = fmt::format("{}", getSetting<int>(name, category));
            break;

        case(E_SettingType::Boolean):
            validValues = fmt::format("true/false");
            defaultValue = getSetting<bool>(name, category) ? "true" : "false";
            break;

        default:
            break;
        }

        ss << fmt::format("|{}|{}|{}|\n", description, validValues, defaultValue);
    }

    return (ss.str());
}

VectorString Settings::getChangedSettings()
{
    VectorString result;

    for(auto& T : settingTypes)
    {
        auto key = T.first;
        std::string name = T.first.second;
        std::string category = T.first.first;

        if(settingIsPrivate[key])
            continue; // Do not include an internal setting

        if(settingActivePriority[key] == 0)
            continue; // Hide setting with default value

        auto label = priorityLabel(static_cast<E_SettingPriority>(settingActivePriority[key]));

        switch(T.second)
        {
        case(E_SettingType::String):
        {
            auto val = getSetting<std::string>(name, category);
            if(val == stringSettingHistory[key][0])
                continue;
            result.push_back(fmt::format("{}.{} = {}  [{}]", category, name, val, label));
            break;
        }
        case(E_SettingType::Double):
        {
            auto val = getSetting<double>(name, category);
            if(val == doubleSettingHistory[key][0])
                continue;
            result.push_back(fmt::format("{}.{} = {}  [{}]", category, name, val, label));
            break;
        }
        case(E_SettingType::Integer):
        {
            auto val = getSetting<int>(name, category);
            if(val == integerSettingHistory[key][0])
                continue;
            result.push_back(fmt::format("{}.{} = {}  [{}]", category, name, val, label));
            break;
        }
        case(E_SettingType::Enum):
        {
            auto val = getSetting<int>(name, category);
            if(val == integerSettingHistory[key][0])
                continue;
            result.push_back(fmt::format("{}.{} = {}  [{}]", category, name, val, label));
            break;
        }
        case(E_SettingType::Boolean):
        {
            auto val = getSetting<bool>(name, category);
            if(val == booleanSettingHistory[key][0])
                continue;
            result.push_back(fmt::format("{}.{} = {}  [{}]", category, name, val, label));
            break;
        }
        default:
            break;
        }
    }

    return (result);
}

VectorString Settings::getSettingIdentifiers(E_SettingType type)
{
    VectorString names;

    for(auto& T : settingTypes)
    {
        auto key = T.first;
        std::string name = T.first.second;
        std::string category = T.first.first;

        if(settingIsPrivate[key])
            continue; // Do not include an internal setting

        if(T.second == type)
            names.push_back(fmt::format("{}.{}", category, name));
    }

    return (names);
}

VectorPairString Settings::getSettingSplitIdentifiers(E_SettingType type)
{
    VectorPairString names;

    for(auto& T : settingTypes)
    {
        auto key = T.first;
        std::string name = T.first.second;
        std::string category = T.first.first;

        if(settingIsPrivate[key])
            continue; // Do not include an internal setting

        if(T.second == type)
            names.push_back(T.first);
    }

    return (names);
}

bool Settings::readSettingsFromOSoL(std::string osol)
{
    using namespace tinyxml2;

    output->outputTrace(" Starting conversion of settings from OSoL.");

    XMLDocument osolDocument;

    auto result = osolDocument.Parse(osol.c_str());

    if(result != XML_SUCCESS)
    {
        output->outputError("  Could not parse options in OSoL-format.", std::to_string(result));
        return (false);
    }

    auto osolNode
        = osolDocument.FirstChildElement("osol")->FirstChildElement("optimization")->FirstChildElement("solverOptions");

    if(osolNode == nullptr)
    {
        output->outputError("  No solver options specified in OSoL-file.");
        return (false);
    }

    for(auto N = osolNode->FirstChildElement("solverOption"); N != nullptr; N = N->NextSiblingElement("solverOption"))
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

            if(settingTypes.find(key) == settingTypes.end())
            {
                output->outputError(
                    "  Cannot update setting <" + category + "," + name + "> since it has not been defined.");

                return (false);
            }

            std::string::size_type convertedChars = value.length();

            switch(settingTypes[key])
            {
            case E_SettingType::String:
                updateSetting(name, category, value, E_SettingPriority::OptionsFile);
                break;
            case E_SettingType::Enum:
            case E_SettingType::Integer:
                try
                {
                    updateSetting(name, category, std::stoi(value, &convertedChars), E_SettingPriority::OptionsFile);
                }
                catch(...)
                {
                    output->outputError("  Cannot update setting <" + category + "," + name
                        + "> since the value is of the wrong format.");
                }
                break;
            case E_SettingType::Boolean:
            {
                bool convertedValue = (value != "false");
                updateSetting(name, category, convertedValue, E_SettingPriority::OptionsFile);
                break;
            }
            case E_SettingType::Double:
                try
                {
                    updateSetting(name, category, std::stod(value), E_SettingPriority::OptionsFile);
                }
                catch(...)
                {
                    output->outputError("  Cannot update setting <" + category + "," + name
                        + "> since the value is of the wrong format.");
                }
                break;
            default:
                break;
            }

            if(convertedChars != value.length())
                output->outputError(
                    "  Cannot update setting <" + name + "," + category + "> since it is of the wrong type.");
        }
        catch(std::exception&)
        {
            output->outputError("  Error when reading OSoL line " + std::to_string(N->GetLineNum()));
            return (false);
        }
    }

    return (true);
}

bool Settings::readSettingsFromString(std::string options)
{
    output->outputTrace(" Starting conversion of settings from GAMS options format.");

    std::istringstream f(options);
    std::string line;

    while(std::getline(f, line))
    {
        // Ignore empty lines and comments (starting with an asterisk)
        if(line == "" || line.at(0) == '*' || line.at(0) == '\r' || line.at(0) == '\n')
            continue;

        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

        int equalitySignIndex = line.find('=');

        std::string key = line.substr(0, equalitySignIndex);
        std::string value = line.substr(equalitySignIndex + 1, line.size());

        int dotIndex = line.find('.');
        std::string category = key.substr(0, dotIndex);
        std::string name = key.substr(dotIndex + 1, line.size());

        if(key == "" || value == "")
        {
            output->outputError("  Error when reading line \"" + line + "\" in the options file; ignoring the option.");
            continue;
        }

        category = Utilities::trim(category);
        name = Utilities::trim(name);
        value = Utilities::trim(value);

        PairString keyPair = make_pair(category, name);

        if(settingTypes.find(keyPair) == settingTypes.end())
        {
            output->outputError(
                "  Cannot update setting <" + name + "," + category + "> since it has not been defined.");

            return (false);
        }

        std::string::size_type convertedChars = value.length();

        switch(settingTypes[keyPair])
        {
        case E_SettingType::String:
            updateSetting(name, category, value, E_SettingPriority::OptionsFile);
            break;
        case E_SettingType::Enum:
        case E_SettingType::Integer:
            try
            {
                updateSetting(name, category, std::stoi(value, &convertedChars), E_SettingPriority::OptionsFile);
            }
            catch(...)
            {
                output->outputError(
                    "  Cannot update setting <" + category + "," + name + "> since the value is of the wrong format.");
            }
            break;
        case E_SettingType::Boolean:
        {
            bool convertedValue = (value != "false");
            updateSetting(name, category, convertedValue, E_SettingPriority::OptionsFile);
            break;
        }
        case E_SettingType::Double:
            try
            {
                updateSetting(name, category, std::stod(value), E_SettingPriority::OptionsFile);
            }
            catch(...)
            {
                output->outputError(
                    "  Cannot update setting <" + category + "," + name + "> since the value is of the wrong format.");
            }
            break;
        default:
            break;
        }

        if(convertedChars != value.length())
            output->outputError(
                "  Cannot update setting <" + name + "," + category + "> since it is of the wrong type.");
    }

    return (true);
}

E_SettingPriority Settings::getSettingPriority(std::string name, std::string category)
{
    PairString key = make_pair(category, name);

    if(settingActivePriority.find(key) == settingActivePriority.end())
    {
        output->outputError("Cannot get priority of setting " + category + "." + name + " since it has not been defined.");
        throw SettingKeyNotFoundException(name, category);
    }

    return static_cast<E_SettingPriority>(settingActivePriority[key]);
}

bool Settings::isSettingAtDefault(std::string name, std::string category)
{
    PairString key = make_pair(category, name);

    if(settingActivePriority.find(key) == settingActivePriority.end())
    {
        output->outputError("Cannot check default state of setting " + category + "." + name + " since it has not been defined.");
        throw SettingKeyNotFoundException(name, category);
    }

    return settingActivePriority[key] == static_cast<int>(E_SettingPriority::Default);
}

VectorString Settings::getSettingsAtPriority(E_SettingPriority priority)
{
    VectorString result;
    int prio = static_cast<int>(priority);

    for(auto& T : settingTypes)
    {
        auto key = T.first;
        std::string name = T.first.second;
        std::string category = T.first.first;

        if(settingIsPrivate[key])
            continue;

        if(settingActivePriority[key] != prio)
            continue;

        switch(T.second)
        {
        case(E_SettingType::String):
            result.push_back(fmt::format("{}.{} = {}", category, name, getSetting<std::string>(name, category)));
            break;
        case(E_SettingType::Double):
            result.push_back(fmt::format("{}.{} = {}", category, name, getSetting<double>(name, category)));
            break;
        case(E_SettingType::Integer):
            result.push_back(fmt::format("{}.{} = {}", category, name, getSetting<int>(name, category)));
            break;
        case(E_SettingType::Enum):
            result.push_back(fmt::format("{}.{} = {}", category, name, getSetting<int>(name, category)));
            break;
        case(E_SettingType::Boolean):
            result.push_back(fmt::format("{}.{} = {}", category, name, getSetting<bool>(name, category)));
            break;
        default:
            break;
        }
    }

    return result;
}

bool Settings::hasSettingAtPriority(std::string name, std::string category, E_SettingPriority priority)
{
    PairString key = make_pair(category, name);
    int prio = static_cast<int>(priority);

    if(settingTypes.find(key) == settingTypes.end())
        return false;

    switch(settingTypes[key])
    {
    case E_SettingType::String:
    {
        auto it = stringSettingHistory.find(key);
        return it != stringSettingHistory.end() && it->second.find(prio) != it->second.end();
    }
    case E_SettingType::Integer:
    case E_SettingType::Enum:
    {
        auto it = integerSettingHistory.find(key);
        return it != integerSettingHistory.end() && it->second.find(prio) != it->second.end();
    }
    case E_SettingType::Double:
    {
        auto it = doubleSettingHistory.find(key);
        return it != doubleSettingHistory.end() && it->second.find(prio) != it->second.end();
    }
    case E_SettingType::Boolean:
    {
        auto it = booleanSettingHistory.find(key);
        return it != booleanSettingHistory.end() && it->second.find(prio) != it->second.end();
    }
    default:
        return false;
    }
}

std::vector<E_SettingPriority> Settings::getSettingPriorityHistory(std::string name, std::string category)
{
    PairString key = make_pair(category, name);

    if(settingTypes.find(key) == settingTypes.end())
    {
        output->outputError("Cannot get priority history of setting " + category + "." + name + " since it has not been defined.");
        throw SettingKeyNotFoundException(name, category);
    }

    std::set<int> prioSet;

    auto collectPrios = [&](auto& histMap) {
        auto it = histMap.find(key);
        if(it != histMap.end())
            for(auto& p : it->second)
                prioSet.insert(p.first);
    };

    switch(settingTypes[key])
    {
    case E_SettingType::String:
        collectPrios(stringSettingHistory);
        break;
    case E_SettingType::Integer:
    case E_SettingType::Enum:
        collectPrios(integerSettingHistory);
        break;
    case E_SettingType::Double:
        collectPrios(doubleSettingHistory);
        break;
    case E_SettingType::Boolean:
        collectPrios(booleanSettingHistory);
        break;
    default:
        break;
    }

    std::vector<E_SettingPriority> result;
    for(int p : prioSet)
        result.push_back(static_cast<E_SettingPriority>(p));
    return result;
}

} // namespace SHOT
