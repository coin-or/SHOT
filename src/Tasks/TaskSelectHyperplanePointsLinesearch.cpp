/*
 * TaskSelectHyperplanePoints.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#include <TaskSelectHyperplanePointsLinesearch.h>

TaskSelectHyperplanePointsLinesearch::TaskSelectHyperplanePointsLinesearch()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("HyperplaneLinesearch");

	if (settings->getIntSetting("LinesearchMethod", "Linesearch") == static_cast<int>(ES_LinesearchMethod::Boost))
	{
		processInfo->logger.message(2) << "Boost linesearch implementation selected" << CoinMessageEol;
		linesearchMethod = new LinesearchMethodBoost();
	}
	else if (settings->getIntSetting("LinesearchMethod", "Linesearch")
			== static_cast<int>(ES_LinesearchMethod::Bisection))
	{
		processInfo->logger.message(2) << "Bisection linesearch selected" << CoinMessageEol;
		linesearchMethod = new LinesearchMethodBisection();
	}
	processInfo->stopTimer("HyperplaneLinesearch");
}

TaskSelectHyperplanePointsLinesearch::~TaskSelectHyperplanePointsLinesearch()
{
	delete linesearchMethod;
}

void TaskSelectHyperplanePointsLinesearch::run()
{
	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration
	auto prevIter = processInfo->getPreviousIteration(); // The already solved iteration

	auto allSolutions = prevIter->solutionPoints; // All solutions in the solution pool
	//TODO add task removing interior points

	auto originalProblem = processInfo->originalProblem;
	for (int i = 0; i < allSolutions.size(); i++)
	{

		if (originalProblem->isConstraintsFulfilledInPoint(allSolutions.at(i).point))
		{
			//processInfo->logger.message(3) << "LP point is on the interior!" << CoinMessageEol;
			// TODO add as primal solution candidate
		}
		else
		{
			for (int j = 0; j < processInfo->interiorPts.size(); j++)
			{
				//processInfo->startTimer(E_TimerTypes::Linesearch);
				auto xNLP = processInfo->interiorPts.at(j).point;

				processInfo->startTimer("HyperplaneLinesearch");
				auto xNewc = linesearchMethod->findZero(xNLP, allSolutions.at(i).point,
						settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
						settings->getDoubleSetting("LinesearchEps", "Linesearch"));

				processInfo->stopTimer("HyperplaneLinesearch");
				//processInfo->stopTimer(E_TimerTypes::Linesearch);

				if (xNewc.size() == 0) break; // TODO remove?

				auto tmpMostDevConstr = originalProblem->getMostDeviatingConstraint(xNewc);

				if (tmpMostDevConstr.value < 0)
				{
					processInfo->logger.message(6) << "Hyperplane point is on the interior." << CoinMessageEol;
					processInfo->addPrimalSolutionCandidate(xNewc, E_PrimalSolutionSource::Linesearch,
							prevIter->iterationNumber);
				}
				else
				{
					processInfo->logger.message(6) << "Hyperplane point is on the exterior." << CoinMessageEol;
					processInfo->addDualSolutionCandidate(xNewc, E_DualSolutionSource::Linesearch,
							prevIter->iterationNumber);

					std::pair<int, std::vector<double>> tmpItem;
					tmpItem.first = tmpMostDevConstr.idx;
					tmpItem.second = xNewc;
					processInfo->hyperplaneWaitingList.push_back(tmpItem);
				}
			}
		}
	}
}

std::string TaskSelectHyperplanePointsLinesearch::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
