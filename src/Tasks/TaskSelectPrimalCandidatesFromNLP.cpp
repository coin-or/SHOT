/*
 * TaskSelectPrimalCandidatesFromNLP.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include "TaskSelectPrimalCandidatesFromNLP.h"

TaskSelectPrimalCandidatesFromNLP::TaskSelectPrimalCandidatesFromNLP()
{
	//processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
	ProcessInfo::getInstance().startTimer("PrimalBoundSearchNLP");

	primalStrategyFixedNLP = new PrimalSolutionStrategyFixedNLP();

	ProcessInfo::getInstance().stopTimer("PrimalBoundSearchNLP");
	ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
}

TaskSelectPrimalCandidatesFromNLP::~TaskSelectPrimalCandidatesFromNLP()
{
	// TODO Auto-generated destructor stub
}

void TaskSelectPrimalCandidatesFromNLP::run()
{

	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->isMILP() && ProcessInfo::getInstance().getRelativeObjectiveGap() > 1e-10)
	{
		ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
		ProcessInfo::getInstance().startTimer("PrimalBoundSearchNLP");
		primalStrategyFixedNLP->runStrategy();

		ProcessInfo::getInstance().stopTimer("PrimalBoundSearchNLP");
		ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
	}
}

std::string TaskSelectPrimalCandidatesFromNLP::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
