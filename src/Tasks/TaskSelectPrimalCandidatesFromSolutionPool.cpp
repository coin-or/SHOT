/*
 * TaskSelectPrimalCandidatesFromSolutionPool.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include "TaskSelectPrimalCandidatesFromSolutionPool.h"

TaskSelectPrimalCandidatesFromSolutionPool::TaskSelectPrimalCandidatesFromSolutionPool()
{

}

TaskSelectPrimalCandidatesFromSolutionPool::~TaskSelectPrimalCandidatesFromSolutionPool()
{
	// TODO Auto-generated destructor stub
}

void TaskSelectPrimalCandidatesFromSolutionPool::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->isMILP()/*
	 && ProcessInfo::getInstance().getRelativeObjectiveGap() > Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")*/)
	{
		ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
		auto allSolutions = ProcessInfo::getInstance().getCurrentIteration()->solutionPoints;

		ProcessInfo::getInstance().addPrimalSolutionCandidates(allSolutions, E_PrimalSolutionSource::MILPSolutionPool);

		ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
	}
}

std::string TaskSelectPrimalCandidatesFromSolutionPool::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

