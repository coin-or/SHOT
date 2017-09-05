#include "TaskInitializeLinesearch.h"

TaskInitializeLinesearch::TaskInitializeLinesearch()
{
	//processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	ProcessInfo::getInstance().startTimer("HyperplaneLinesearch");

	if (settings->getIntSetting("LinesearchMethod", "Linesearch") == static_cast<int>(ES_LinesearchMethod::Bisection))
	{
		ProcessInfo::getInstance().linesearchMethod = new LinesearchMethodBisection();
		ProcessInfo::getInstance().outputInfo("Bisection linesearch implementation selected.");
	}
	else
	{
		ProcessInfo::getInstance().linesearchMethod = new LinesearchMethodBoost();
		ProcessInfo::getInstance().outputInfo("Boost linesearch implementation selected.");
	}

	ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
}

TaskInitializeLinesearch::~TaskInitializeLinesearch()
{
}

void TaskInitializeLinesearch::run()
{

}

std::string TaskInitializeLinesearch::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

