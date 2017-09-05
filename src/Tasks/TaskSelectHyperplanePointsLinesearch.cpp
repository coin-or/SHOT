/*
 * TaskSelectHyperplanePoints.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#include "TaskSelectHyperplanePointsLinesearch.h"

TaskSelectHyperplanePointsLinesearch::TaskSelectHyperplanePointsLinesearch()
{
	//processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskSelectHyperplanePointsLinesearch::~TaskSelectHyperplanePointsLinesearch()
{
}

void TaskSelectHyperplanePointsLinesearch::run()
{
	this->run(ProcessInfo::getInstance().getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsLinesearch::run(vector<SolutionPoint> solPoints)
{
	int addedHyperplanes = 0;

	auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

	auto originalProblem = ProcessInfo::getInstance().originalProblem;

	int prevHPnum = ProcessInfo::getInstance().hyperplaneWaitingList.size();

	for (int i = 0; i < solPoints.size(); i++)
	{
		if (originalProblem->isConstraintsFulfilledInPoint(solPoints.at(i).point))
		{
		}
		else
		{
			for (int j = 0; j < ProcessInfo::getInstance().interiorPts.size(); j++)
			{
				if (addedHyperplanes >= settings->getIntSetting("MaxHyperplanesPerIteration", "Algorithm")) return;
				auto xNLP = ProcessInfo::getInstance().interiorPts.at(j).point;

				std::vector<double> externalPoint;
				std::vector<double> internalPoint;

				try
				{

					ProcessInfo::getInstance().startTimer("HyperplaneLinesearch");
					auto xNewc = ProcessInfo::getInstance().linesearchMethod->findZero(xNLP, solPoints.at(i).point,
							settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
							settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"),
							settings->getDoubleSetting("LinesearchConstrEps", "Linesearch"));

					ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
					internalPoint = xNewc.first;
					externalPoint = xNewc.second;
				}
				catch (std::exception &e)
				{
					ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
					externalPoint = solPoints.at(i).point;

					ProcessInfo::getInstance().outputWarning(
							"     Cannot find solution with linesearch, using solution point instead.");
				}

				auto tmpMostDevConstr = originalProblem->getMostDeviatingConstraint(externalPoint);

				if (tmpMostDevConstr.value >= 0)
				{
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

					ProcessInfo::getInstance().hyperplaneWaitingList.push_back(hyperplane);
					addedHyperplanes++;

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
