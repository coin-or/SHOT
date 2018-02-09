#include "TaskPresolve.h"

TaskPresolve::TaskPresolve(IMIPSolver *MIPSolver)
{

	isPresolved = false;
	this->MIPSolver = MIPSolver;
}

TaskPresolve::~TaskPresolve()
{
	// TODO Auto-generated destructor stub
}

void TaskPresolve::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	auto strategy = static_cast<ES_MIPPresolveStrategy>(Settings::getInstance().getIntSetting("MIP.Presolve.Frequency", "Dual"));

	if (!currIter->isMIP())
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
	MIPSolver->setTimeLimit(timeLim);

	if (ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
		MIPSolver->setCutOff(ProcessInfo::getInstance().getPrimalBound());
	}

	if (MIPSolver->getDiscreteVariableStatus() && ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
		MIPSolver->addMIPStart(ProcessInfo::getInstance().primalSolution);
	}

	if (Settings::getInstance().getBoolSetting("FixedInteger.UsePresolveBounds", "Primal") || Settings::getInstance().getBoolSetting("MIP.Presolve.UpdateObtainedBounds", "Dual"))
	{
		MIPSolver->presolveAndUpdateBounds();
		isPresolved = true;
	}
}

std::string TaskPresolve::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
