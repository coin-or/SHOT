#include "SHOTSettings.h"

//#include <stdexcept>

extern const OSSmartPtr<OSOutput> osoutput;

enum ESettingsType
{
	String, Integer, Double, Enum, Boolean
};
std::map<std::pair<std::string, std::string>, std::string> _settings;

typedef std::map<std::pair<std::string, std::string>, std::string>::iterator SettingsIter;
SettingsIter _settingsIter;

std::map<std::pair<std::string, std::string>, std::string> _settingsDesc;
std::map<std::pair<std::string, std::string>, ESettingsType> _settingsType;
std::map<std::pair<std::string, std::string>, bool> _isPrivate;
std::map<std::pair<std::string, std::string>, std::pair<double, double>> _settingsBounds;
std::map<std::pair<std::string, std::string>, bool> _settingsEnum;
std::map<std::tuple<std::string, std::string, int>, std::string> _enumDescription;

typedef std::map<std::tuple<std::string, std::string, int>, std::string>::iterator EnumDescriptionIter;
EnumDescriptionIter _enumDescriptionIter;

void Settings::updateSettingBase(std::pair<std::string, std::string> key, std::string value)
{
	std::string oldvalue = _settings[key];

	if (oldvalue == value)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info,
				"Setting " + key.first + "." + key.second + " not updated. Same value " + oldvalue + " given.");
		return;
	}
	else
	{
		_settings[key] = value;

		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info,
				"Setting " + key.first + "." + key.second + " = " + oldvalue + " updated. New value = " + value + ".");
	}
}

void Settings::createSetting(std::string name, std::string category, std::string value, std::string description)
{
	createSetting(name, category, value, description, false);
}

void Settings::createSetting(std::string name, std::string category, std::string value, std::string description,
		bool isPrivate)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settings[key] = value;
	_settingsDesc[key] = description;
	_settingsType[key] = ESettingsType::String;
	std::string test = name;
	_isPrivate[key] = isPrivate;

	osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info,
			"Setting <" + name + "," + category + "> = " + value + " created.");
}

void Settings::updateSetting(std::string name, std::string category, std::string value)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settingsIter = _settings.find(key);

	if (_settingsIter == _settings.end())
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot update setting <" + name + "," + category + "> since it has not been defined.");

		throw SettingKeyNotFoundException(name, category);
	}

	if (_settingsType[key] != ESettingsType::String)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot update setting <" + name + "," + category
						+ "> since it is of the wrong type. (Expected string).");
		throw SettingSetWrongTypeException(name, category);
	}

	updateSettingBase(key, value);
}

std::string Settings::getStringSetting(std::string name, std::string category)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settingsIter = _settings.find(key);

	if (_settingsIter == _settings.end())
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get setting <" + name + "," + category + "> since it has not been defined.");
		throw SettingKeyNotFoundException(name, category);
	}

	if (_settingsType[key] != ESettingsType::String)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get value of setting <" + name + "," + category + "> as string: Wrong type requested!");
		throw SettingGetWrongTypeException(name, category);
	}

	return _settings[key];
}

//Integer settings ==============================================================

void Settings::createSetting(std::string name, std::string category, int value, std::string description, double minVal,
		double maxVal, bool isPrivate)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settings[key] = boost::lexical_cast < std::string > (value);
	_settingsDesc[key] = description;
	_settingsType[key] = ESettingsType::Integer;
	_settingsBounds[key] = std::make_pair(minVal, maxVal);
	_isPrivate[key] = isPrivate;

	osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info,
			"Setting <" + name + "," + category + "> = " + to_string(value) + " created.");

}

void Settings::createSetting(std::string name, std::string category, int value, std::string description, double minVal,
		double maxVal)
{
	Settings::createSetting(name, category, value, description, minVal, maxVal, false);
}

void Settings::createSetting(std::string name, std::string category, int value, std::string description)
{
	Settings::createSetting(name, category, value, description, -OSDBL_MAX, OSDBL_MAX, false);
}

void Settings::updateSetting(std::string name, std::string category, int value)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settingsIter = _settings.find(key);

	if (_settingsIter == _settings.end())
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot update setting <" + name + "," + category + "> since it has not been defined.");
		throw SettingKeyNotFoundException(name, category);
	}

	if (_settingsType[key] != ESettingsType::Integer)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot set value of setting <" + name + "," + category + "> as integer: Wrong type!");
		throw SettingSetWrongTypeException(name, category);
	}

	if (_settingsBounds[key].first > value || _settingsBounds[key].second < value)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot update setting <" + name + "," + category + ">: Not in interval ["
						+ to_string(_settingsBounds[key].first) + "," + to_string(_settingsBounds[key].second) + "].");

		throw SettingOutsideBoundsException(name, category, (double) value, _settingsBounds[key].first,
				_settingsBounds[key].second);
	}

	std::string newvalue = boost::lexical_cast < std::string > (value);

	updateSettingBase(key, newvalue);
}

int Settings::getIntSetting(std::string name, std::string category)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settingsIter = _settings.find(key);

	if (_settingsIter == _settings.end())
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get setting <" + name + "," + category + "> since it has not been defined.");
		throw SettingKeyNotFoundException(name, category);
	}

	if (_settingsType[key] != ESettingsType::Integer)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get value of setting <" + name + "," + category + "> as integer: Wrong type requested!");
		throw SettingGetWrongTypeException(name, category);
	}

	try
	{
		int intval = boost::lexical_cast<int>(_settings[key]);
		return intval;
	}
	catch (boost::bad_lexical_cast &e)
	{

		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get value of setting <" + name + "," + category + "> as integer: Wrong type requested!");
		throw SettingGetWrongTypeException(name, category);
	}
}

//Boolean settings ==============================================================

void Settings::createSetting(std::string name, std::string category, bool value, std::string description,
		bool isPrivate)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settings[key] = boost::lexical_cast < std::string > (value);
	_settingsDesc[key] = description;
	_settingsType[key] = ESettingsType::Boolean;
	_isPrivate[key] = isPrivate;

	osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info,
			"Setting <" + name + "," + category + "> = " + to_string(value) + " created.");

}

void Settings::createSetting(std::string name, std::string category, bool value, std::string description)
{
	Settings::createSetting(name, category, value, description, false);
}

void Settings::updateSetting(std::string name, std::string category, bool value)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settingsIter = _settings.find(key);

	if (_settingsIter == _settings.end())
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot update setting <" + name + "," + category + "> since it has not been defined.");
		throw SettingKeyNotFoundException(name, category);
	}

	if (_settingsType[key] != ESettingsType::Boolean)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot set value of setting <" + name + "," + category + "> as bool: Wrong type!");
		throw SettingSetWrongTypeException(name, category);
	}

	std::string newvalue = boost::lexical_cast < std::string > (value);

	updateSettingBase(key, newvalue);
}

bool Settings::getBoolSetting(std::string name, std::string category)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settingsIter = _settings.find(key);

	if (_settingsIter == _settings.end())
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get setting <" + name + "," + category + "> since it has not been defined.");
		throw SettingKeyNotFoundException(name, category);
	}

	if (_settingsType[key] != ESettingsType::Boolean)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get value of setting <" + name + "," + category + "> as boolean: Wrong type requested!");
		throw SettingGetWrongTypeException(name, category);
	}

	try
	{
		bool boolval = boost::lexical_cast<bool>(_settings[key]);
		return boolval;
	}
	catch (boost::bad_lexical_cast &e)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get value of setting <" + name + "," + category + "> as boolean: Wrong type requested!");
		throw SettingGetWrongTypeException(name, category);
	}
}

//Enum settings ==================================================================

void Settings::createSetting(std::string name, std::string category, int value, std::string description,
		std::vector<std::string> enumDescriptions, bool isPrivate)
{
	createSetting(name, category, value, description, -OSDBL_MAX, OSDBL_MAX, isPrivate);

	for (int i = 0; i < enumDescriptions.size(); i++)
	{
		_enumDescription[make_tuple(name, category, i)] = enumDescriptions.at(i);

		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info,
				" Enum value " + to_string(i) + ": " + enumDescriptions.at(i));
	}

	_settingsEnum[make_pair(name, category)] = true;
}

void Settings::createSetting(std::string name, std::string category, int value, std::string description,
		std::vector<std::string> enumDescriptions)
{
	Settings::createSetting(name, category, value, description, enumDescriptions, false);
}

std::string Settings::getEnumDescriptionList(std::string name, std::string category)
{
	std::stringstream desc;

	for (EnumDescriptionIter iterator = _enumDescription.begin(); iterator != _enumDescription.end(); iterator++)
	{
		std::tuple<std::string, std::string, int> t = iterator->first;

		if (name == std::get < 0 > (t) && category == std::get < 1 > (t))
		{
			desc << boost::lexical_cast < std::string > (std::get < 2 > (t)) << ": " << iterator->second << ". ";
		}
	}

	return desc.str();
}

std::string Settings::getEnumDescription(std::string name, std::string category)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settingsIter = _settings.find(key);

	if (_settingsIter == _settings.end())
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get setting <" + name + "," + category + "> since it has not been defined.");
		throw SettingKeyNotFoundException(name, category);
	}

	if (_settingsType[key] != ESettingsType::Integer)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get value of setting <" + name + "," + category + "> as integer: Wrong type requested!");

		throw SettingGetWrongTypeException(name, category);
	}

	try
	{
		int intval = boost::lexical_cast<int>(_settings[key]);
		std::tuple<std::string, std::string, int> tpl = make_tuple(name, category, intval);
		return _enumDescription[tpl];
	}
	catch (boost::bad_lexical_cast &e)
	{

		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get value of setting <" + name + "," + category + "> as integer: Wrong type requested!");

		throw SettingGetWrongTypeException(name, category);
	}
}

//Double settings ================================================================

void Settings::createSetting(std::string name, std::string category, double value, std::string description)
{
	Settings::createSetting(name, category, value, description, -OSDBL_MAX, OSDBL_MAX, false);
}

void Settings::createSetting(std::string name, std::string category, double value, std::string description,
		double minVal, double maxVal)
{
	Settings::createSetting(name, category, value, description, -OSDBL_MAX, OSDBL_MAX, false);
}

void Settings::createSetting(std::string name, std::string category, double value, std::string description,
		double minVal, double maxVal, bool isPrivate)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settings[key] = boost::lexical_cast < std::string > (value);
	_settingsDesc[key] = description;
	_settingsType[key] = ESettingsType::Double;
	_settingsBounds[key] = std::make_pair(minVal, maxVal);
	_isPrivate[key] = isPrivate;

	osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info,
			"Setting <" + name + "," + category + "> = " + to_string(value) + " created.");

}

void Settings::updateSetting(std::string name, std::string category, double value)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settingsIter = _settings.find(key);

	if (_settingsIter == _settings.end())
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot update setting <" + name + "," + category + "> since it has not been defined.");
		throw SettingKeyNotFoundException(name, category);
	}

	if (_settingsType[key] != ESettingsType::Double)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot set value of setting <" + name + "," + category + "> as double: Wrong type!");

		throw SettingSetWrongTypeException(name, category);
	}

	if (_settingsBounds[key].first > value || _settingsBounds[key].second < value)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot update setting <" + name + "," + category + ">: Not in interval ["
						+ to_string(_settingsBounds[key].first) + "," + to_string(_settingsBounds[key].second) + "].");

		throw SettingOutsideBoundsException(name, category, value, _settingsBounds[key].first,
				_settingsBounds[key].second);
	}

	std::string newvalue = boost::lexical_cast < std::string > (value);

	updateSettingBase(key, newvalue);
}

double Settings::getDoubleSetting(std::string name, std::string category)
{
	std::pair < std::string, std::string > key = make_pair(name, category);
	_settingsIter = _settings.find(key);

	if (_settingsIter == _settings.end())
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get setting <" + name + "," + category + "> since it has not been defined.");
		throw SettingKeyNotFoundException(name, category);
	}

	if (_settingsType[key] != ESettingsType::Double)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get value of setting <" + name + "," + category + "> as double: Wrong type requested!");
		throw SettingGetWrongTypeException(name, category);
	}

	try
	{
		double intval = boost::lexical_cast<double>(_settings[key]);
		return intval;
	}
	catch (boost::bad_lexical_cast &e)
	{
		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
				"Cannot get value of setting <" + name + "," + category + "> as double: Wrong type requested!");

		throw SettingGetWrongTypeException(name, category);
	}
}

struct SortPred
{
		bool operator()(const boost::property_tree::ptree::value_type &v1,
				const boost::property_tree::ptree::value_type &v2) const
		{
			if (v1.first == "solverOption" && v2.first == "solverOption")
			{
				if (v1.second.get < std::string > ("<xmlattr>.category")
						== v2.second.get < std::string > ("<xmlattr>.category"))
				{
					return (v1.second.get < std::string > ("<xmlattr>.name") < v2.second.get < std::string
							> ("<xmlattr>.name"));
				}
				else
				{
					return (v1.second.get < std::string > ("<xmlattr>.category") < v2.second.get < std::string
							> ("<xmlattr>.category"));
				}
			}

			return (v1.first < v2.first);
		}
};

std::string Settings::getSettingsInOSolFormat()
{
	OSoLWriter *osolwriter = new OSoLWriter();
	osolwriter->m_bWhiteSpace = false;

	boost::property_tree::ptree pt;
	boost::property_tree::xml_writer_settings < std::string > settings(' ', 1);

	stringstream ss;
	ss << osolwriter->writeOSoL(getSettingsAsOSOption());

	read_xml(ss, pt, boost::property_tree::xml_parser::trim_whitespace);

	// This sort the options according to category first and name after
	pt.get_child("osol.optimization.solverOptions").sort(SortPred());

	std::ostringstream oss;
	write_xml(oss, pt, settings);

	delete osolwriter;

	return (oss.str());
}

std::string Settings::getSettingsAsString()
{
	OSoLWriter *osolwriter = new OSoLWriter();
	osolwriter->m_bWhiteSpace = false;

	boost::property_tree::ptree pt;

	stringstream ss;

	auto osOption = getSettingsAsOSOption();
	ss << osolwriter->writeOSoL(osOption);
	delete osOption;

	read_xml(ss, pt, boost::property_tree::xml_parser::trim_whitespace);

	// This sort the options according to category first and name after
	pt.get_child("osol.optimization.solverOptions").sort(SortPred());

	auto tmp = pt.get_child("osol.optimization.solverOptions");

	std::ostringstream oss;

	for (auto& child : pt.get_child("osol.optimization.solverOptions"))
	{
		if (child.second.get < std::string > ("<xmlattr>.name", "") == "") continue;
		oss << (((int) oss.tellp()) == 0 ? "[" : ", [");
		oss << child.second.get < std::string > ("<xmlattr>.category", "") << ",";
		oss << child.second.get < std::string > ("<xmlattr>.name", "") << "] = ";
		oss << child.second.get < std::string > ("<xmlattr>.value", "");
	}

	delete osolwriter;

	return (oss.str());
}

OSOption* Settings::getSettingsAsOSOption()
{
	osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info,
			"Starting conversion of settings to OSOption object.");

	OSOption* options = new OSOption();

	for (SettingsIter iterator = _settings.begin(); iterator != _settings.end(); iterator++)
	{
		auto p = iterator->first;

		if (_isPrivate[p]) continue;	// Do not include an internal setting in the class

		std::stringstream type;

		switch (_settingsType[p])
		{
			case ESettingsType::String:
				type << "string";
				break;
			case ESettingsType::Integer:
				type << "integer";
				break;
			case ESettingsType::Boolean:
				type << "integer";
				break;
			case ESettingsType::Enum:
				type << "integer";
				break;
			case ESettingsType::Double:
				type << "double";
		}

		std::stringstream desc;

		if (_settingsEnum[p] == true)
		{
			desc << _settingsDesc[p] << ": " << getEnumDescriptionList(p.first, p.second);
		}
		else
		{
			desc << _settingsDesc[p] << ". ";
		}

		options->setAnotherSolverOption(p.first, iterator->second, "SHOT", p.second, type.str(), desc.str());

		osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info,
				" Setting <" + p.first + "," + p.second + "> converted.");

		type.clear();
		desc.clear();
	}

	osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info,
			"Conversion of settings to OSOption object completed.");

	return options;
}

std::string Settings::getSettingsInGAMSOptFormat()
{
	OSoLWriter *osolwriter = new OSoLWriter();
	osolwriter->m_bWhiteSpace = false;

	boost::property_tree::ptree pt;

	stringstream ss;
	ss << osolwriter->writeOSoL(getSettingsAsOSOption());

	read_xml(ss, pt, boost::property_tree::xml_parser::trim_whitespace);

	// This sort the options according to category first and name after
	pt.get_child("osol.optimization.solverOptions").sort(SortPred());

	auto tmp = pt.get_child("osol.optimization.solverOptions");

	std::ostringstream oss;

	for (auto& child : pt.get_child("osol.optimization.solverOptions"))
	{
		std::stringstream desc;

		std::string name = child.second.get < std::string > ("<xmlattr>.name", "");
		std::string category = child.second.get < std::string > ("<xmlattr>.category", "");
		std::string value = child.second.get < std::string > ("<xmlattr>.value", "");

		auto key = make_pair(name, category);

		if (_settingsEnum[make_pair(name, category)] == true)
		{
			desc << _settingsDesc[key] << ": " << getEnumDescriptionList(name, category);
		}
		else
		{
			desc << _settingsDesc[key] << ". ";
		}

		if (name == "") continue;
		oss << std::endl;
		if ("desc" != "") oss << "* " << desc.str() << std::endl;
		oss << category << ".";
		oss << name << " = ";
		oss << value;
		oss << std::endl;
	}

	delete osolwriter;

	return (oss.str());
}

void Settings::readSettingsFromOSoL(std::string osol)
{

	osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info, "Starting conversion of settings from OSoL.");

	OSoLReader *osolreader = new OSoLReader();

	readSettingsFromOSOption(osolreader->readOSoL(osol));

	delete osolreader;
}

void Settings::readSettingsFromGAMSOptFormat(std::string options)
{
	osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info,
			"Starting conversion of settings from GAMS options format.");

	std::istringstream f(options);
	std::string line;

	while (std::getline(f, line))
	{
		//Ignore empty lines and comments (starting with an asterisk)
		if (line == "" || boost::algorithm::starts_with(line, "*")) continue;

		std::vector < string > nameCategoryPair;
		std::vector < string > keyValuePair;
		boost::split(keyValuePair, line, boost::is_any_of("="));
		//boost::split(nameCategoryPair, keyValuePair.front(), std::bind1st(std::equal_to<char>(), '.'));

		int dotindex = line.find('.');
		std::string category = keyValuePair.at(0).substr(0, dotindex); 
		std::string name = keyValuePair.at(0).substr(dotindex+1,line.size());

		if (keyValuePair.size() != 2)
		{
			osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
					"Error when reading line \"" + line + "\" in the options file; ignoring the option.");

			continue;
		}

		//std::string category = nameCategoryPair.at(0);
		//std::string name = nameCategoryPair.at(1);
		std::string value = keyValuePair.at(1);

		boost::trim(category);
		boost::trim(name);
		boost::trim(value);

		std::pair < std::string, std::string > key = make_pair(name, category);
		_settingsIter = _settings.find(key);

		if (_settingsIter == _settings.end())
		{
			osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
					"Cannot update setting <" + name + "," + category + "> since it has not been defined.");

			throw SettingKeyNotFoundException(name, category);
		}

		try
		{
			switch (_settingsType[key])
			{
				case ESettingsType::String:
					updateSetting(name, category, value);
					break;
				case ESettingsType::Integer:
					updateSetting(name, category, boost::lexical_cast<int>(value));
					break;
				case ESettingsType::Boolean:
					updateSetting(name, category, boost::lexical_cast<bool>(value));
					break;
				case ESettingsType::Double:
					updateSetting(name, category, boost::lexical_cast<double>(value));
					break;
				case ESettingsType::Enum:
					updateSetting(name, category, boost::lexical_cast<int>(value));
					break;
				default:
					break;
			}
		}
		catch (boost::bad_lexical_cast &)
		{
			osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
					"Cannot update setting <" + name + "," + category + "> since it is of the wrong type.");
		}
	}
}

void Settings::readSettingsFromOSOption(OSOption* options)
{
	osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info, "Conversion of settings from OSOptions.");

	for (int i = 0; i < options->getNumberOfSolverOptions(); i++)
	{
		SolverOption *so = options->getAllSolverOptions()[i];

		if (so->solver == "SHOT")
		{
			try
			{
				if (so->type == "string")
				{
					updateSetting(so->name, so->category, so->value);
				}
				else if (so->type == "integer")
				{
					std::pair < std::string, std::string > key = make_pair(so->name, so->category);
					_settingsIter = _settings.find(key);

					if (_settingsIter == _settings.end())
					{
						throw SettingKeyNotFoundException(so->name, so->category);
					}

					try
					{
						if (_settingsType[key] == ESettingsType::Boolean)
						{
							bool boolval = boost::lexical_cast<bool>(so->value);
							updateSetting(so->name, so->category, boolval);
						}
						else // Integer type
						{
							int intval = boost::lexical_cast<int>(so->value);
							updateSetting(so->name, so->category, intval);
						}
					}
					catch (boost::bad_lexical_cast &e)
					{

						osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
								"Value for setting <" + so->name + "," + so->category
										+ "> in OSoL file is not an integer. Using default value.");
					}
				}
				else if (so->type == "double")
				{
					try
					{
						double dblval = boost::lexical_cast<double>(so->value);
						updateSetting(so->name, so->category, dblval);
					}
					catch (boost::bad_lexical_cast &e)
					{
						osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
								"Value for setting <" + so->name + "," + so->category
										+ "> in OSoL file is not a double. Using default value.");
					}
				}
				else
				{

					osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error,
							"Value for setting <" + so->name + "," + so->category
									+ "> in OSoL file is of unknown type. Skipping it.");
				}
			}
			catch (SettingKeyNotFoundException &e)
			{
			}
			catch (SettingSetWrongTypeException &e)
			{
			}
			catch (SettingOutsideBoundsException &e)
			{
			}
		}
	}

	osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info, "Conversion of settings from OSoL completed.");
}
