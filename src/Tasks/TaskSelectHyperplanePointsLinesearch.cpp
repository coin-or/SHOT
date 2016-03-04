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

	int prevHPnum = processInfo->hyperplaneWaitingList.size();

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

				std::vector<double> externalPoint;
				std::vector<double> internalPoint;

				try
				{

					processInfo->startTimer("HyperplaneLinesearch");
					auto xNewc = linesearchMethod->findZero(xNLP, allSolutions.at(i).point,
							settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
							settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"),
							settings->getDoubleSetting("LinesearchConstrEps", "Linesearch"));

					processInfo->stopTimer("HyperplaneLinesearch");
					internalPoint = xNewc.first;
					externalPoint = xNewc.second;

					//processInfo->addPrimalSolutionCandidate(internalPoint, E_PrimalSolutionSource::Linesearch,
					//		processInfo->getCurrentIteration()->iterationNumber);

				}
				catch (std::exception &e)
				{
					processInfo->stopTimer("HyperplaneLinesearch");
					externalPoint = allSolutions.at(i).point;

					processInfo->logger.message(0)
							<< "Cannot find solution with linesearch using solution point instead: "
							<< CoinMessageNewline << e.what() << CoinMessageEol;
				}

				auto tmpMostDevConstr = originalProblem->getMostDeviatingConstraint(externalPoint);

				if (tmpMostDevConstr.value >= 0)
				{
					processInfo->logger.message(6) << "Hyperplane point is on the exterior." << CoinMessageEol;

					Hyperplane hyperplane;
					hyperplane.sourceConstraintIndex = tmpMostDevConstr.idx;
					hyperplane.generatedPoint = externalPoint;

					if (i == 0 && currIter->isMILP())
					{
						hyperplane.source = E_HyperplaneSource::MIPOptimalLinesearch;
					}
					else if (currIter->isMILP())
					{
						hyperplane.source = E_HyperplaneSource::MIPSolutionPoolLinesearch;
					}
					else
					{
						hyperplane.source = E_HyperplaneSource::LPRelaxedLinesearch;
					}

					processInfo->hyperplaneWaitingList.push_back(hyperplane);

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
