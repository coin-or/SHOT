#include "RelaxationStrategyStandard.h"

RelaxationStrategyStandard::RelaxationStrategyStandard()
{
	//MILPSolver = solver;
	//relaxationActive = false;

	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

}

RelaxationStrategyStandard::~RelaxationStrategyStandard()
{
}

void RelaxationStrategyStandard::setInitial()
{
	LPFinished = false;

	if (settings->getIntSetting("IterLimitLP", "Algorithm") > 0)
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
	if (isLPStepFinished() || isCurrentToleranceReached() || isIterationLimitReached()
			|| isRelaxedSolutionEpsilonValid() || isObjectiveStagnant())
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
	if (processInfo->MILPSolver->getDiscreteVariableStatus())
	{
		processInfo->stopTimer("MILP");
		processInfo->startTimer("LP");
		processInfo->MILPSolver->activateDiscreteVariables(false);

		processInfo->getCurrentIteration()->type = E_IterationProblemType::Relaxed;

	}
}

void RelaxationStrategyStandard::setInactive()
{
	if (!processInfo->MILPSolver->getDiscreteVariableStatus())
	{
		processInfo->stopTimer("LP");
		processInfo->startTimer("MILP");
		processInfo->MILPSolver->activateDiscreteVariables(true);

		processInfo->getCurrentIteration()->type = E_IterationProblemType::MIP;

		LPFinished = true;

	}
}

E_IterationProblemType RelaxationStrategyStandard::getProblemType()
{
	if (processInfo->MILPSolver->getDiscreteVariableStatus())

	return E_IterationProblemType::MIP;
	else return E_IterationProblemType::Relaxed;
}

bool RelaxationStrategyStandard::isIterationLimitReached()
{
	auto prevIter = processInfo->getPreviousIteration();

	if (prevIter->iterationNumber < settings->getIntSetting("IterLimitLP", "Algorithm"))
	{
		return false;
	}

	return true;
}

bool RelaxationStrategyStandard::isLPStepFinished()
{
	return LPFinished;
}

bool RelaxationStrategyStandard::isObjectiveStagnant()
{
	int numSteps = 10;

	auto prevIter = processInfo->getPreviousIteration();

	if (prevIter->iterationNumber < numSteps) return false;

	auto prevIter2 = &processInfo->iterations[prevIter->iterationNumber - numSteps];

	//TODO: should be substituted with parameter
	if (std::abs((prevIter->objectiveValue - prevIter2->objectiveValue) / prevIter->objectiveValue) < 0.000001) return true;

	return false;
}
