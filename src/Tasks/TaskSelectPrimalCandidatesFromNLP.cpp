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

	if (currIter->isMILP() && processInfo->getRelativeObjectiveGap() > 1e-10)
	{
		processInfo->startTimer("PrimalBoundTotal");
		processInfo->startTimer("PrimalBoundSearchNLP");
		primalStrategyFixedNLP->runStrategy();

		processInfo->stopTimer("PrimalBoundSearchNLP");
		processInfo->stopTimer("PrimalBoundTotal");
	}
}

std::string TaskSelectPrimalCandidatesFromNLP::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
