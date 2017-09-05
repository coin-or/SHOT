#include "RelaxationStrategyBase.h"

/*RelaxationStrategyBase::RelaxationStrategyBase()
 {
 }


 RelaxationStrategyBase::~RelaxationStrategyBase()
 {
 }*/

bool RelaxationStrategyBase::isRelaxedSolutionEpsilonValid()
{
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (prevIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	//if (currIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyBase::isRelaxedSolutionInterior()
{
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (prevIter->maxDeviation < 0)
	//if (currIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyBase::isCurrentToleranceReached()
{
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (prevIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolLP", "Algorithm"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyBase::isGapReached()
{
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (ProcessInfo::getInstance().getAbsoluteObjectiveGap() < 2 * settings->getDoubleSetting("GapTermTolAbsolute", "Algorithm"))
	{
		return true;
	}

	if (ProcessInfo::getInstance().getRelativeObjectiveGap() < 2 * settings->getDoubleSetting("GapTermTolRelative", "Algorithm"))
	{
		return true;
	}

	return false;
}
