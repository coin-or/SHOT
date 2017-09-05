#include "RelaxationStrategyStandard.h"

RelaxationStrategyStandard::RelaxationStrategyStandard(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;
	//processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

}

RelaxationStrategyStandard::~RelaxationStrategyStandard()
{
}

void RelaxationStrategyStandard::setInitial()
{
	LPFinished = false;

	if (settings->getIntSetting("IterLimitLP", "Algorithm") > 0
			&& settings->getDoubleSetting("TimeLimitLP", "Algorithm") > 0)
	{
		this->setActive();
	}
	else
	{
		this->setInactive();
	}
}

void RelaxationStrategyStandard::executeStrategy()
{
	int iterInterval = settings->getIntSetting("IterSolveLPRelaxation", "Algorithm");
	if (iterInterval != 0 && ProcessInfo::getInstance().getCurrentIteration()->iterationNumber % iterInterval == 0)
	{
		return (this->setActive());

	}

	if (isLPStepFinished() || isCurrentToleranceReached() || isGapReached() || isIterationLimitReached()
			|| isTimeLimitReached() || isRelaxedSolutionEpsilonValid() || isObjectiveStagnant())
	{
		this->setInactive();
	}
	else
	{
		this->setActive();
	}
}

void RelaxationStrategyStandard::setActive()
{
	if (MILPSolver->getDiscreteVariableStatus())
	{
		ProcessInfo::getInstance().stopTimer("MILP");
		ProcessInfo::getInstance().startTimer("LP");
		MILPSolver->activateDiscreteVariables(false);

		ProcessInfo::getInstance().getCurrentIteration()->type = E_IterationProblemType::Relaxed;

	}
}

void RelaxationStrategyStandard::setInactive()
{
	if (!MILPSolver->getDiscreteVariableStatus())
	{
		ProcessInfo::getInstance().stopTimer("LP");
		ProcessInfo::getInstance().startTimer("MILP");
		MILPSolver->activateDiscreteVariables(true);

		ProcessInfo::getInstance().getCurrentIteration()->type = E_IterationProblemType::MIP;

		LPFinished = true;

	}
}

E_IterationProblemType RelaxationStrategyStandard::getProblemType()
{
	if (MILPSolver->getDiscreteVariableStatus())

	return (E_IterationProblemType::MIP);
	else return (E_IterationProblemType::Relaxed);
}

bool RelaxationStrategyStandard::isIterationLimitReached()
{
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (prevIter->iterationNumber < settings->getIntSetting("IterLimitLP", "Algorithm"))
	{
		return (false);
	}

	return (true);
}

bool RelaxationStrategyStandard::isTimeLimitReached()
{
	if (ProcessInfo::getInstance().getElapsedTime("LP") < settings->getDoubleSetting("TimeLimitLP", "Algorithm"))
	{
		return (false);
	}

	return (true);
}

bool RelaxationStrategyStandard::isLPStepFinished()
{

	return (LPFinished);
}

bool RelaxationStrategyStandard::isObjectiveStagnant()
{
	int numSteps = 10;

	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (prevIter->iterationNumber < numSteps) return (false);

	auto prevIter2 = &ProcessInfo::getInstance().iterations[prevIter->iterationNumber - numSteps];

//TODO: should be substituted with parameter
	if (std::abs((prevIter->objectiveValue - prevIter2->objectiveValue) / prevIter->objectiveValue) < 0.000001) return (true);

	return (false);
}

