/*
 * TaskSelectHyperplanePointsSolution.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#include <TaskSelectHyperplanePointsSolution.h>

TaskSelectHyperplanePointsSolution::TaskSelectHyperplanePointsSolution()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskSelectHyperplanePointsSolution::~TaskSelectHyperplanePointsSolution()
{

}

void TaskSelectHyperplanePointsSolution::run()
{
	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration
	auto prevIter = processInfo->getPreviousIteration(); // The already solved iteration

	auto allSolutions = prevIter->solutionPoints; // All solutions in the solution pool
	auto originalProblem = processInfo->originalProblem;

	for (int i = 0; i < allSolutions.size(); i++)
	{
		auto tmpMostDevConstr = originalProblem->getMostDeviatingConstraint(allSolutions.at(i).point);

		if (tmpMostDevConstr.value < 0)
		{
			processInfo->logger.message(3) << "LP point is on the interior!" << CoinMessageEol;
			// TODO add as primal solution candidate
		}
		else
		{
			std::pair<int, std::vector<double>> tmpItem;
			tmpItem.first = tmpMostDevConstr.idx;
			tmpItem.second = allSolutions.at(i).point;

			processInfo->hyperplaneWaitingList.push_back(tmpItem);
		}
	}
}
