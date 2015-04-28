#include "RelaxationStrategyAdaptive.h"

RelaxationStrategyAdaptive::RelaxationStrategyAdaptive()
{
	//processInfo->MILPSolver = solver;
	//relaxationActive = false;

	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

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
	if (settings->getIntSetting("IterLimitLP", "Algorithm") > 0)
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

	//if (processInfo->getPreviousIteration()->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
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
	if (processInfo->MILPSolver->getDiscreteVariableStatus())
	{
		processInfo->stopTimer("MILP");
		processInfo->startTimer("LP");
		processInfo->MILPSolver->activateDiscreteVariables(false);

		processInfo->getCurrentIteration()->type = E_IterationProblemType::Relaxed;

	}
}

void RelaxationStrategyAdaptive::setInactive()
{
	if (!processInfo->MILPSolver->getDiscreteVariableStatus())
	{
		processInfo->stopTimer("LP");
		processInfo->startTimer("MILP");
		processInfo->MILPSolver->activateDiscreteVariables(true);

		processInfo->getCurrentIteration()->type = E_IterationProblemType::MIP;
	}
}

E_IterationProblemType RelaxationStrategyAdaptive::getProblemType()
{
	if (processInfo->MILPSolver->getDiscreteVariableStatus()) return E_IterationProblemType::MIP;
	else return E_IterationProblemType::Relaxed;
}

bool RelaxationStrategyAdaptive::isRelaxationDistanceSmall()
{
	/*if (processInfo->iterations.size() <= 2)
	 {
	 return false;
	 }

	 if (processInfo->getCurrentIteration()->hyperplanePoints.size() == 0 || processInfo->getPreviousIteration()->hyperplanePoints.size() == 0)
	 {
	 return true;
	 }

	 auto currIterSol = processInfo->getCurrentIteration()->hyperplanePoints[0];
	 auto prevIterSol = processInfo->getPreviousIteration()->hyperplanePoints[0];

	 double distance = calculateDistance(currIterSol, prevIterSol);
	 */

	if (processInfo->getPreviousIteration()->boundaryDistance < currentDistanceLevel && currentDistanceLevel < DBL_MAX)
	{

		return true;
	}

	return false;
}

bool RelaxationStrategyAdaptive::isCurrentToleranceReached()
{
	auto currIter = processInfo->getCurrentIteration();

	if (currIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolLP", "Algorithm"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyAdaptive::isIterationLimitReached()
{
	auto currIter = processInfo->getCurrentIteration();

	if (currIter->iterationNumber - iterLastMILP == settings->getIntSetting("IterLimitLP", "Algorithm"))
	{
		iterLastMILP = currIter->iterationNumber;
		return true;
	}

	return false;
}

void RelaxationStrategyAdaptive::updateCurrentDistanceLevel()
{
	if (processInfo->getPreviousIteration()->boundaryDistance < DBL_MAX)
	{
		double tmpVal = 0.0;

		distanceLevels.push_back(processInfo->getPreviousIteration()->boundaryDistance);
		int numVals = distanceLevels.size();

		int considerVals = std::min(numVals, 5);

		for (double i = numVals; i > numVals - considerVals; i--)
		{
			//tmpVal = (1.0 / (considerVals+1.0))*tmpVal*distanceLevels[i-1];
			tmpVal = tmpVal + distanceLevels[i - 1];
		}

		//currentDistanceLevel = 0.5*pow(tmpVal, 1.0 / considerVals);
		//processInfo->logger.message(1) << " TMPval " << tmpVal << CoinMessageEol;
		currentDistanceLevel = 0.5 * tmpVal / considerVals;
		//processInfo->logger.message(1) << " Updated distance level " << currentDistanceLevel << CoinMessageEol;
		//auto tmp = processInfo->getPreviousIteration()->boundaryDistance;

		//currentDistanceLevel = currentDistanceLevel - pow(processInfo->getPreviousIteration()->boundaryDistance / initialDistanceLevel, 1.0 / numLPMeans);

		//numLPMeans = numLPMeans + 1.0;
	}
}
