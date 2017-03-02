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
	int addedHyperplanes = 0;

	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration
	auto prevIter = processInfo->getPreviousIteration(); // The already solved iteration

	auto allSolutions = prevIter->solutionPoints; // All solutions in the solution pool
	auto originalProblem = processInfo->originalProblem;

	for (int i = 0; i < allSolutions.size(); i++)
	{
		if (addedHyperplanes >= settings->getIntSetting("MaxHyperplanesPerIteration", "Algorithm")) return;

		auto tmpMostDevConstr = originalProblem->getMostDeviatingConstraint(allSolutions.at(i).point);

		if (tmpMostDevConstr.value < 0)
		{
			processInfo->outputWarning("LP point is on the interior!");
		}
		else
		{
			Hyperplane hyperplane;
			hyperplane.sourceConstraintIndex = tmpMostDevConstr.idx;
			hyperplane.generatedPoint = allSolutions.at(i).point;

			if (i == 0 && currIter->isMILP())
			{
				hyperplane.source = E_HyperplaneSource::MIPOptimalSolutionPoint;
			}
			else if (currIter->isMILP())
			{
				hyperplane.source = E_HyperplaneSource::MIPSolutionPoolSolutionPoint;
			}
			else
			{
				hyperplane.source = E_HyperplaneSource::LPRelaxedSolutionPoint;
			}

			processInfo->hyperplaneWaitingList.push_back(hyperplane);

			addedHyperplanes++;
		}
	}
}

std::string TaskSelectHyperplanePointsSolution::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
