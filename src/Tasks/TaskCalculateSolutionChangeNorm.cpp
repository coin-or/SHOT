/*
 * TaskCalculateSolutionChangeNorm.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#include <TaskCalculateSolutionChangeNorm.h>

TaskCalculateSolutionChangeNorm::TaskCalculateSolutionChangeNorm()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskCalculateSolutionChangeNorm::~TaskCalculateSolutionChangeNorm()
{
	// TODO Auto-generated destructor stub
}

void TaskCalculateSolutionChangeNorm::run()
{
	auto currIter = processInfo->getCurrentIteration();

	currIter->boundaryDistance = DBL_MAX;

	if (processInfo->iterations.size() < 3)
	{
		return;
	}

	if (processInfo->getCurrentIteration()->hyperplanePoints.size() == 0
			|| processInfo->getCurrentIteration()->isMILP())
	{
		return;
	}

	auto currIterSol = processInfo->getCurrentIteration()->hyperplanePoints.at(0);

	for (int i = processInfo->iterations.size() - 2; i >= 1; i--)
	{
		if (processInfo->iterations.size() > 0 && !processInfo->iterations.at(i).isMILP())
		{
			auto prevIterSol = processInfo->iterations.at(i).hyperplanePoints.at(0);

			double distance = 0;

			for (int j = 0; j < currIterSol.size(); j++)
			{
				distance = distance + (currIterSol.at(j) - prevIterSol.at(j)) * (currIterSol.at(j) - prevIterSol.at(j));
			}

			distance = sqrt(distance + 0.001);

			if (OSIsnan(distance)) // Checks for INF, do not remove!
			{
				currIter->boundaryDistance = DBL_MAX;
			}
			else
			{
				currIter->boundaryDistance = distance;
			}

			return;

		}
	}

}

std::string TaskCalculateSolutionChangeNorm::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
