#include "TaskExecuteRelaxationStrategy.h"

TaskExecuteRelaxationStrategy::TaskExecuteRelaxationStrategy()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	auto solver = processInfo->MILPSolver;

	if (settings->getIntSetting("RelaxationStrategy", "Algorithm") == static_cast<int>(ES_RelaxationStrategy::Adaptive))
	{
		relaxationStrategy = new RelaxationStrategyAdaptive();
	}
	else if (settings->getIntSetting("RelaxationStrategy", "Algorithm")
			== static_cast<int>(ES_RelaxationStrategy::Standard))
	{
		relaxationStrategy = new RelaxationStrategyStandard();
	}
	else if (settings->getIntSetting("RelaxationStrategy", "Algorithm")
			== static_cast<int>(ES_RelaxationStrategy::None))
	{
		relaxationStrategy = new RelaxationStrategyNone();
	}

	processInfo->relaxationStrategy = relaxationStrategy;

	isInitialized = false;

}

TaskExecuteRelaxationStrategy::~TaskExecuteRelaxationStrategy()
{
	delete relaxationStrategy;
}

void TaskExecuteRelaxationStrategy::run()
{
	if (!isInitialized)
	{
		relaxationStrategy->setInitial();
		isInitialized = true;
	}
	else
	{

		relaxationStrategy->executeStrategy();
	}

}
