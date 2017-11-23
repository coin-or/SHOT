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

	if (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	//if (currIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyBase::isRelaxedSolutionInterior()
{
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (prevIter->maxDeviation < 0)
	//if (currIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyBase::isCurrentToleranceReached()
{
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstrTermTolLP", "Algorithm"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyBase::isGapReached()
{
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (ProcessInfo::getInstance().getAbsoluteObjectiveGap()
			< 2 * Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm"))
	{
		return true;
	}

	if (ProcessInfo::getInstance().getRelativeObjectiveGap()
			< 2 * Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm"))
	{
		return true;
	}

	return false;
}
