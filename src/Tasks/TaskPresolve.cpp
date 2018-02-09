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

	auto strategy = static_cast<ES_MIPPresolveStrategy>(Settings::getInstance().getIntSetting("MIP.Presolve.Frequency", "Dual"));

	if (!currIter->isMILP())
	{
		return;
	}

	if (strategy == ES_MIPPresolveStrategy::Never)
	{
		return;
	}
	else if (strategy == ES_MIPPresolveStrategy::Once && isPresolved == true)
	{
		return;
	}

	// Sets the iteration time limit
	auto timeLim = Settings::getInstance().getDoubleSetting("TimeLimit", "Termination") - ProcessInfo::getInstance().getElapsedTime("Total");
	MILPSolver->setTimeLimit(timeLim);

	if (ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
		MILPSolver->setCutOff(ProcessInfo::getInstance().getPrimalBound());
	}

	if (MILPSolver->getDiscreteVariableStatus() && ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
		MILPSolver->addMIPStart(ProcessInfo::getInstance().primalSolution);
	}

	if (Settings::getInstance().getBoolSetting("FixedInteger.UsePresolveBounds", "Primal") || Settings::getInstance().getBoolSetting("MIP.Presolve.UpdateObtainedBounds", "Dual"))
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
