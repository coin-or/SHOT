/*
 * TaskSelectPrimalCandidatesFromSolutionPool.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include <TaskSelectPrimalCandidatesFromSolutionPool.h>

TaskSelectPrimalCandidatesFromSolutionPool::TaskSelectPrimalCandidatesFromSolutionPool()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskSelectPrimalCandidatesFromSolutionPool::~TaskSelectPrimalCandidatesFromSolutionPool()
{
	// TODO Auto-generated destructor stub
}

void TaskSelectPrimalCandidatesFromSolutionPool::run()
{
	auto currIter = processInfo->getCurrentIteration();

	if (currIter->isMILP() && processInfo->getRelativeObjectiveGap() > 1e-10)
	{
		processInfo->startTimer("PrimalBoundTotal");
		auto allSolutions = processInfo->getCurrentIteration()->solutionPoints;
		processInfo->addPrimalSolutionCandidates(allSolutions, E_PrimalSolutionSource::MILPSolutionPool);

		processInfo->stopTimer("PrimalBoundTotal");
	}
}
