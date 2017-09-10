#include "TaskPresolve.h"

TaskPresolve::TaskPresolve(IMILPSolver *MILPSolver)
{

	isPresolved = false;
	this->MILPSolver = MILPSolver;
}

TaskPresolve::~TaskPresolve()
{
	// TODO Auto-generated destructor stub
}

void TaskPresolve::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	auto strategy = static_cast<ES_PresolveStrategy>(Settings::getInstance().getIntSetting("PresolveStrategy",
			"Presolve"));

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
	auto timeLim = Settings::getInstance().getDoubleSetting("TimeLimit", "Algorithm")
			- ProcessInfo::getInstance().getElapsedTime("Total");
	MILPSolver->setTimeLimit(timeLim);

	if (ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
		MILPSolver->setCutOff(ProcessInfo::getInstance().getPrimalBound());
	}

	if (MILPSolver->getDiscreteVariableStatus() && ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
		MILPSolver->addMIPStart(ProcessInfo::getInstance().primalSolution);
	}

	if (Settings::getInstance().getBoolSetting("UsePresolveBoundsForPrimalNLP", "Presolve")
			|| Settings::getInstance().getBoolSetting("UsePresolveBoundsForMIP", "Presolve"))
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
