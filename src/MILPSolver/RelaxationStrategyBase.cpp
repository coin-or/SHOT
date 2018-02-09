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

	if (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"))
	//if (currIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyBase::isRelaxedSolutionInterior()
{
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (prevIter->maxDeviation < 0)
	//if (currIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyBase::isCurrentToleranceReached()
{
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("Relaxation.TerminationTolerance", "Dual"))
	{
		return true;
	}

	return false;
}

bool RelaxationStrategyBase::isGapReached()
{
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (ProcessInfo::getInstance().getAbsoluteObjectiveGap()
			< 2 * Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination"))
	{
		return true;
	}

	if (ProcessInfo::getInstance().getRelativeObjectiveGap()
			< 2 * Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination"))
	{
		return true;
	}

	return false;
}
