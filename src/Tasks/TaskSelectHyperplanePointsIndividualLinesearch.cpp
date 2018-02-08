/*
 * TaskSelectHyperplanePointsIndividualLinesearch.cpp
 *
 *  Created on: Feb 3, 2017
 *      Author: alundell
 */

#include "TaskSelectHyperplanePointsIndividualLinesearch.h"

TaskSelectHyperplanePointsIndividualLinesearch::TaskSelectHyperplanePointsIndividualLinesearch()
{

	nonlinearConstraintIdxs = ProcessInfo::getInstance().originalProblem->getNonlinearConstraintIndexes();

	ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
}

TaskSelectHyperplanePointsIndividualLinesearch::~TaskSelectHyperplanePointsIndividualLinesearch()
{
}

void TaskSelectHyperplanePointsIndividualLinesearch::run()
{
	this->run(ProcessInfo::getInstance().getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsIndividualLinesearch::run(vector<SolutionPoint> solPoints)
{
	int addedHyperplanes = 0;

	auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

	// Contains boolean array that indicates if a constraint has been added or not
	std::vector<bool> hyperplaneAddedToConstraint(
		ProcessInfo::getInstance().originalProblem->getNumberOfNonlinearConstraints(), false);

	for (int i = 0; i < solPoints.size(); i++)
	{
		auto maxDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(
			solPoints.at(i).point);

		if (maxDevConstr.value <= 0)
		{
			continue;
		}
		else
		{
			for (int j = 0; j < ProcessInfo::getInstance().interiorPts.size(); j++)
			{
				auto xNLP = ProcessInfo::getInstance().interiorPts.at(j)->point;

				for (int k = 0; k < nonlinearConstraintIdxs.size(); k++)
				{
					int currConstrIdx = nonlinearConstraintIdxs.at(k);

					// Do not add hyperplane if one has been added for this constraint already
					if ((currConstrIdx != -1 && hyperplaneAddedToConstraint.at(k)) || ((currConstrIdx == -1) && hyperplaneAddedToConstraint.back()))
						continue;

					auto constrDevExterior =
						ProcessInfo::getInstance().originalProblem->calculateConstraintFunctionValue(currConstrIdx,
																									 solPoints.at(i).point);

					if (addedHyperplanes >= Settings::getInstance().getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual"))
						return;

					// Do not add hyperplane if less than this tolerance or negative
					if (constrDevExterior < Settings::getInstance().getDoubleSetting("ESH.Linesearch.ConstraintTolerance", "Dual"))
						continue;

					// Do not add hyperplane if constraint value is much less than largest
					if (constrDevExterior < Settings::getInstance().getDoubleSetting("ESH.Linesearch.ConstraintFactor", "Dual") * maxDevConstr.value)
						continue;

					std::vector<double> externalPoint;
					std::vector<double> internalPoint;

					std::vector<int> currentIndexes;

					currentIndexes.push_back(currConstrIdx);

					try
					{
						ProcessInfo::getInstance().startTimer("HyperplaneLinesearch");
						auto xNewc = ProcessInfo::getInstance().linesearchMethod->findZero(xNLP, solPoints.at(i).point,
																						   Settings::getInstance().getIntSetting("LinesearchMaxIter", "Linesearch"),
																						   Settings::getInstance().getDoubleSetting("LinesearchLambdaEps", "Linesearch"),
																						   Settings::getInstance().getDoubleSetting("LinesearchConstrEps", "Linesearch"),
																						   currentIndexes);

						ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
						internalPoint = xNewc.first;
						externalPoint = xNewc.second;
					}
					catch (std::exception &e)
					{
						ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
						externalPoint = solPoints.at(i).point;

						ProcessInfo::getInstance().outputError(
							"     Cannot find solution with linesearch. Interior value: " + to_string(ProcessInfo::getInstance().interiorPts.at(j)->maxDevatingConstraint.value) + " exterior value: " + to_string(constrDevExterior));
					}

					auto constrDevBoundary =
						ProcessInfo::getInstance().originalProblem->calculateConstraintFunctionValue(currConstrIdx,
																									 externalPoint);

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

						ProcessInfo::getInstance().hyperplaneWaitingList.push_back(hyperplane);
						addedHyperplanes++;

						if (currConstrIdx != -1)
							hyperplaneAddedToConstraint.at(k) = true;
						else
							hyperplaneAddedToConstraint.at(hyperplaneAddedToConstraint.back()) = true;

						ProcessInfo::getInstance().outputInfo(
							"     Added hyperplane to constraint " + to_string(currConstrIdx) + " original dev: " + to_string(constrDevExterior));
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
