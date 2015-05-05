/*
 * TaskSelectPrimalCandidatesFromNLP.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include <TaskSelectPrimalCandidatesFromNLP.h>

TaskSelectPrimalCandidatesFromNLP::TaskSelectPrimalCandidatesFromNLP()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("PrimalBoundTotal");
	processInfo->startTimer("PrimalBoundSearchNLP");

	primalStrategyFixedNLP = new PrimalSolutionStrategyFixedNLP();

	primalStrategyFixedNLP->createProblem(processInfo->originalProblem->getProblemInstance());

	processInfo->stopTimer("PrimalBoundSearchNLP");
	processInfo->stopTimer("PrimalBoundTotal");
}

TaskSelectPrimalCandidatesFromNLP::~TaskSelectPrimalCandidatesFromNLP()
{
	// TODO Auto-generated destructor stub
}

void TaskSelectPrimalCandidatesFromNLP::run()
{
	auto currIter = processInfo->getCurrentIteration();

	if (currIter->isMILP() && processInfo->getRelativeObjectiveGap() > 1e-10
	/*&& processInfo->getElapsedTime("Total") <= settings->getDoubleSetting("TimeLimit", "Algorithm")*/)
	{
		processInfo->startTimer("PrimalBoundTotal");
		processInfo->startTimer("PrimalBoundSearchNLP");

		primalStrategyFixedNLP->runStrategy();

		processInfo->stopTimer("PrimalBoundSearchNLP");
		processInfo->stopTimer("PrimalBoundTotal");
	}
}
