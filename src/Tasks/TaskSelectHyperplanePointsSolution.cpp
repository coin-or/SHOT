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

	auto constrSelFactor = settings->getDoubleSetting("LinesearchConstraintSelectionFactor", "ECP");

	for (int i = 0; i < allSolutions.size(); i++)
	{
		auto tmpMostDevConstrs = originalProblem->getMostDeviatingConstraints(allSolutions.at(i).point,
				constrSelFactor);

		for (int j = 0; j < tmpMostDevConstrs.size(); j++)
		{
			if (addedHyperplanes >= settings->getIntSetting("MaxHyperplanesPerIteration", "Algorithm")) return;

			if (tmpMostDevConstrs.at(j).value < 0)
			{
				processInfo->outputWarning("LP point is in the interior!");
			}
			else
			{
				Hyperplane hyperplane;
				hyperplane.sourceConstraintIndex = tmpMostDevConstrs.at(j).idx;
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
}

std::string TaskSelectHyperplanePointsSolution::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
