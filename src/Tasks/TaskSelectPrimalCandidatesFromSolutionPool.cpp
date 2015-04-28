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

	processInfo->startTimer("PrimalBoundTotal");
	auto allSolutions = processInfo->getCurrentIteration()->variableSolutions;
	processInfo->addPrimalSolutionCandidates(allSolutions, E_PrimalSolutionSource::MILPSolutionPool,
			processInfo->getCurrentIteration()->iterationNumber);

	processInfo->stopTimer("PrimalBoundTotal");
}
