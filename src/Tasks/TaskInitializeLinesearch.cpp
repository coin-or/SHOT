#include "TaskInitializeLinesearch.h"

TaskInitializeLinesearch::TaskInitializeLinesearch()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("HyperplaneLinesearch");

	if (settings->getIntSetting("LinesearchMethod", "Linesearch") == static_cast<int>(ES_LinesearchMethod::Bisection))
	{
		processInfo->linesearchMethod = new LinesearchMethodBisection();
		processInfo->outputInfo("Bisection linesearch implementation selected.");
	}
	else
	{
		processInfo->linesearchMethod = new LinesearchMethodBoost();
		processInfo->outputInfo("Boost linesearch implementation selected.");
	}

	processInfo->stopTimer("HyperplaneLinesearch");
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

