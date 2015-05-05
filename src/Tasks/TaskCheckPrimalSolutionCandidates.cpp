/*
 * TaskCheckPrimalSolutionCandidates.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include <TaskCheckPrimalSolutionCandidates.h>

TaskCheckPrimalSolutionCandidates::TaskCheckPrimalSolutionCandidates()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("PrimalBoundTotal");
	primalStrategyCheckPoint = new PrimalSolutionStrategyCheckPoint();

	processInfo->stopTimer("PrimalBoundTotal");

}

TaskCheckPrimalSolutionCandidates::~TaskCheckPrimalSolutionCandidates()
{
	delete primalStrategyCheckPoint;
}

void TaskCheckPrimalSolutionCandidates::run()
{

	processInfo->startTimer("PrimalBoundTotal");

	for (auto cand : processInfo->primalSolutionCandidates)
	{

		primalStrategyCheckPoint->checkPoint(cand);
		//std::cout << "Check!" << std::endl;

	}

	processInfo->primalSolutionCandidates.clear();

	processInfo->stopTimer("PrimalBoundTotal");

}
