#include "TaskInitializeLinesearch.h"

TaskInitializeLinesearch::TaskInitializeLinesearch()
{

	ProcessInfo::getInstance().startTimer("HyperplaneLinesearch");

	if (Settings::getInstance().getIntSetting("Rootsearch.Method", "Subsolver")
			== static_cast<int>(ES_LinesearchMethod::Bisection))
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

