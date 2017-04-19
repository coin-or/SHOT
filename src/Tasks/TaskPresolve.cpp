#include <TaskPresolve.h>

TaskPresolve::TaskPresolve()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	isPresolved = false;
}

TaskPresolve::~TaskPresolve()
{
	// TODO Auto-generated destructor stub
}

void TaskPresolve::run()
{
	auto currIter = processInfo->getCurrentIteration();
	auto MILPSolver = processInfo->MILPSolver;

	auto strategy = static_cast<ES_PresolveStrategy>(settings->getIntSetting("PresolveStrategy", "Presolve"));

	if (!currIter->isMILP())
	{
		return;
	}

	if (strategy == ES_PresolveStrategy::Never)
	{
		return;
	}
	else if (strategy == ES_PresolveStrategy::Once && isPresolved == true)
	{
		return;
	}

	// Sets the iteration time limit
	auto timeLim = settings->getDoubleSetting("TimeLimit", "Algorithm") - processInfo->getElapsedTime("Total");
	MILPSolver->setTimeLimit(timeLim);

	if (processInfo->primalSolutions.size() > 0)
	{
		MILPSolver->setCutOff(processInfo->getPrimalBound());
	}

	if (MILPSolver->getDiscreteVariableStatus() && processInfo->primalSolutions.size() > 0)
	{
		MILPSolver->addMIPStart(processInfo->primalSolution);
	}

	if (settings->getBoolSetting("UsePresolveBoundsForPrimalNLP", "Presolve")
			|| settings->getBoolSetting("UsePresolveBoundsForMIP", "Presolve"))
	{
		MILPSolver->presolveAndUpdateBounds();
		isPresolved = true;
	}
}

std::string TaskPresolve::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
