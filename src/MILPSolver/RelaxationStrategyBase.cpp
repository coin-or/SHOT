#include "RelaxationStrategyBase.h"

/*RelaxationStrategyBase::RelaxationStrategyBase()
{
}


RelaxationStrategyBase::~RelaxationStrategyBase()
{
}*/



bool RelaxationStrategyBase::isRelaxedSolutionEpsilonValid()
{
	auto prevIter = processInfo->getPreviousIteration();
	
	if (prevIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
		//if (currIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyBase::isRelaxedSolutionInterior()
{
	auto prevIter = processInfo->getPreviousIteration();

	if (prevIter->maxDeviation < 0)
		//if (currIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		return true;
	}

	return false;
}


bool RelaxationStrategyBase::isCurrentToleranceReached()
{
	auto prevIter = processInfo->getPreviousIteration();

	if (prevIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolLP", "Algorithm"))
	{
		return true;
	}

	return false;
}