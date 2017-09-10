#include "RelaxationStrategyAdaptive.h"

RelaxationStrategyAdaptive::RelaxationStrategyAdaptive(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;

	currentDistanceLevel = 1.0;
	//initialDistanceLevel = 1.0;
	maxLPToleranceReached = false;
	iterLastMILP = 0;
	//numLPMeans = 0.0;
}

RelaxationStrategyAdaptive::~RelaxationStrategyAdaptive()
{
}

void RelaxationStrategyAdaptive::setInitial()
{
	if (Settings::getInstance().getIntSetting("IterLimitLP", "Algorithm") > 0)
	{
		this->setActive();
	}
	else
	{
		this->setInactive();
	}
}

void RelaxationStrategyAdaptive::executeStrategy()
{
	//if (maxLPToleranceReached)
	//{
	//	this->setInactive();
	//	return;
	//}

	//if (ProcessInfo::getInstance().getPreviousIteration()->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
	//{
	//	//this->setInactive();
	//	return;
	//}

	updateCurrentDistanceLevel();

	if (isIterationLimitReached())
	{
		this->setInactive();
		return;
	}

	/*if (MILPSolver->getDiscreteVariableStatus())
	 {
	 this->setActive();
	 return;
	 }*/

	if (isRelaxedSolutionEpsilonValid())
	{
		//maxLPToleranceReached = true;
		this->setInactive();
		return;
	}

	if (isRelaxationDistanceSmall())
	{
		this->setInactive();
		return;
	}

	this->setActive();

}

void RelaxationStrategyAdaptive::setActive()
{
	if (MILPSolver->getDiscreteVariableStatus())
	{
		ProcessInfo::getInstance().stopTimer("MILP");
		ProcessInfo::getInstance().startTimer("LP");
		MILPSolver->activateDiscreteVariables(false);

		ProcessInfo::getInstance().getCurrentIteration()->type = E_IterationProblemType::Relaxed;

	}
}

void RelaxationStrategyAdaptive::setInactive()
{
	if (!MILPSolver->getDiscreteVariableStatus())
	{
		ProcessInfo::getInstance().stopTimer("LP");
		ProcessInfo::getInstance().startTimer("MILP");
		MILPSolver->activateDiscreteVariables(true);

		ProcessInfo::getInstance().getCurrentIteration()->type = E_IterationProblemType::MIP;
	}
}

E_IterationProblemType RelaxationStrategyAdaptive::getProblemType()
{
	if (MILPSolver->getDiscreteVariableStatus()) return E_IterationProblemType::MIP;
	else return E_IterationProblemType::Relaxed;
}

bool RelaxationStrategyAdaptive::isRelaxationDistanceSmall()
{
	/*if (ProcessInfo::getInstance().iterations.size() <= 2)
	 {
	 return false;
	 }

	 if (ProcessInfo::getInstance().getCurrentIteration()->hyperplanePoints.size() == 0 || ProcessInfo::getInstance().getPreviousIteration()->hyperplanePoints.size() == 0)
	 {
	 return true;
	 }

	 auto currIterSol = ProcessInfo::getInstance().getCurrentIteration()->hyperplanePoints[0];
	 auto prevIterSol = ProcessInfo::getInstance().getPreviousIteration()->hyperplanePoints[0];

	 double distance = calculateDistance(currIterSol, prevIterSol);
	 */

	if (ProcessInfo::getInstance().getPreviousIteration()->boundaryDistance < currentDistanceLevel
			&& currentDistanceLevel < OSDBL_MAX)
	{

		return true;
	}

	return false;
}

bool RelaxationStrategyAdaptive::isCurrentToleranceReached()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstrTermTolLP", "Algorithm"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyAdaptive::isIterationLimitReached()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->iterationNumber - iterLastMILP == Settings::getInstance().getIntSetting("IterLimitLP", "Algorithm"))
	{
		iterLastMILP = currIter->iterationNumber;
		return true;
	}

	return false;
}

void RelaxationStrategyAdaptive::updateCurrentDistanceLevel()
{
	if (ProcessInfo::getInstance().getPreviousIteration()->boundaryDistance < OSDBL_MAX)
	{
		double tmpVal = 0.0;

		distanceLevels.push_back(ProcessInfo::getInstance().getPreviousIteration()->boundaryDistance);
		int numVals = distanceLevels.size();

		int considerVals = std::min(numVals, 5);

		for (double i = numVals; i > numVals - considerVals; i--)
		{
			tmpVal = tmpVal + distanceLevels[i - 1];
		}

		currentDistanceLevel = 0.5 * tmpVal / considerVals;
	}
}
