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

	nonlinearConstraintIdxs = processInfo->originalProblem->getNonlinearConstraintIndexes();

	processInfo->stopTimer("HyperplaneLinesearch");
}

TaskSelectHyperplanePointsIndividualLinesearch::~TaskSelectHyperplanePointsIndividualLinesearch()
{
}

void TaskSelectHyperplanePointsIndividualLinesearch::run()
{
	this->run(processInfo->getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsIndividualLinesearch::run(vector<SolutionPoint> solPoints)
{
	int addedHyperplanes = 0;

	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration

	// Contains boolean array that indicates if a constraint has been added or not
	std::vector<bool> hyperplaneAddedToConstraint(processInfo->originalProblem->getNumberOfNonlinearConstraints(),
			false);

	for (int i = 0; i < solPoints.size(); i++)
	{
		auto maxDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(solPoints.at(i).point);

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
							currConstrIdx, solPoints.at(i).point);

					if (addedHyperplanes >= settings->getIntSetting("MaxHyperplanesPerIteration", "Algorithm")) return;

					// Do not add hyperplane if less than this tolerance or negative
					if (constrDevExterior < settings->getDoubleSetting("LinesearchConstraintTolerance", "ESH")) continue;

					// Do not add hyperplane if constraint value is much less than largest
					if (constrDevExterior
							< settings->getDoubleSetting("LinesearchConstraintFactor", "ESH") * maxDevConstr.value) continue;

					std::vector<double> externalPoint;
					std::vector<double> internalPoint;

					std::vector<int> currentIndexes;

					currentIndexes.push_back(currConstrIdx);

					try
					{
						processInfo->startTimer("HyperplaneLinesearch");
						auto xNewc = processInfo->linesearchMethod->findZero(xNLP, solPoints.at(i).point,
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
						externalPoint = solPoints.at(i).point;

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
						addedHyperplanes++;

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

