/*
 * TaskSelectHyperplanePointsIndividualLinesearch.cpp
 *
 *  Created on: Feb 3, 2017
 *      Author: alundell
 */

#include <TaskSelectHyperplanePointsIndividualLinesearch.h>

TaskSelectHyperplanePointsIndividualLinesearch::TaskSelectHyperplanePointsIndividualLinesearch()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("HyperplaneLinesearch");

	if (settings->getIntSetting("LinesearchMethod", "Linesearch") == static_cast<int>(ES_LinesearchMethod::Boost))
	{
		linesearchMethod = new LinesearchMethodBoost();
		processInfo->outputDebug("Boost linesearch implementation selected.");
	}
	else if (settings->getIntSetting("LinesearchMethod", "Linesearch")
			== static_cast<int>(ES_LinesearchMethod::Bisection))
	{
		linesearchMethod = new LinesearchMethodBisection();
		processInfo->outputDebug("Bisection linesearch implementation selected.");
	}

	nonlinearConstraintIdxs = processInfo->originalProblem->getNonlinearConstraintIndexes();

	processInfo->stopTimer("HyperplaneLinesearch");
}

TaskSelectHyperplanePointsIndividualLinesearch::~TaskSelectHyperplanePointsIndividualLinesearch()
{
	delete linesearchMethod;
}

void TaskSelectHyperplanePointsIndividualLinesearch::run()
{
	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration
	auto prevIter = processInfo->getPreviousIteration(); // The already solved iteration

	auto allSolutions = prevIter->solutionPoints; // All solutions in the solution pool

	// Contains boolean array that indicates if a constraint has been added or not
	std::vector<bool> hyperplaneAddedToConstraint(processInfo->originalProblem->getNumberOfNonlinearConstraints(),
			false);

	for (int i = 0; i < allSolutions.size(); i++)
	{
		auto maxDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(allSolutions.at(i).point);

		if (maxDevConstr.value <= 0)
		{
			continue;
		}
		else
		{
			for (int j = 0; j < processInfo->interiorPts.size(); j++)
			{
				auto xNLP = processInfo->interiorPts.at(j).point;

				for (int k = 0; k < nonlinearConstraintIdxs.size(); k++)
				{
					int currConstrIdx = nonlinearConstraintIdxs.at(k);

					// Do not add hyperplane if one has been added for this constraint already
					if ((currConstrIdx != -1 && hyperplaneAddedToConstraint.at(k))
							|| ((currConstrIdx == -1) && hyperplaneAddedToConstraint.back())) continue;

					auto constrDevExterior = processInfo->originalProblem->calculateConstraintFunctionValue(
							currConstrIdx, allSolutions.at(i).point);

					if (constrDevExterior < 0) continue;

					// Do not add hyperplane if constraint value is much less than largest
					if (constrDevExterior <= 0.9 * maxDevConstr.value) continue;

					std::vector<double> externalPoint;
					std::vector<double> internalPoint;

					std::vector<int> currentIndexes;

					currentIndexes.push_back(currConstrIdx);

					try
					{
						processInfo->startTimer("HyperplaneLinesearch");
						auto xNewc = linesearchMethod->findZero(xNLP, allSolutions.at(i).point,
								settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
								settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"),
								settings->getDoubleSetting("LinesearchConstrEps", "Linesearch"), currentIndexes);

						processInfo->stopTimer("HyperplaneLinesearch");
						internalPoint = xNewc.first;
						externalPoint = xNewc.second;
					}
					catch (std::exception &e)
					{
						processInfo->stopTimer("HyperplaneLinesearch");
						externalPoint = allSolutions.at(i).point;

						processInfo->outputError(
								"     Cannot find solution with linesearch. Interior value: "
										+ to_string(processInfo->interiorPts.at(j).maxDevatingConstraint.value)
										+ " exterior value: " + to_string(constrDevExterior));

					}

					auto constrDevBoundary = processInfo->originalProblem->calculateConstraintFunctionValue(
							currConstrIdx, externalPoint);

					if (constrDevBoundary >= 0)
					{
						Hyperplane hyperplane;
						hyperplane.sourceConstraintIndex = currConstrIdx;
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

						if (currConstrIdx != -1) hyperplaneAddedToConstraint.at(k) = true;
						else hyperplaneAddedToConstraint.at(hyperplaneAddedToConstraint.back()) = true;

						processInfo->outputInfo(
								"     Added hyperplane to constraint " + to_string(currConstrIdx) + " original dev: "
										+ to_string(constrDevExterior));

					}
				}
			}
		}
	}

}

std::string TaskSelectHyperplanePointsIndividualLinesearch::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
