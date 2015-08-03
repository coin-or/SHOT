/*
 * TaskAddHyperplanes.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#include <TaskAddHyperplanes.h>

TaskAddHyperplanes::TaskAddHyperplanes()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	itersWithoutAddedHPs = 0;
}

TaskAddHyperplanes::~TaskAddHyperplanes()
{
	// TODO Auto-generated destructor stub
}

void TaskAddHyperplanes::run()
{
	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration

	if (!currIter->isMILP() || !settings->getBoolSetting("DelayedConstraints", "MILP")
			|| !currIter->MILPSolutionLimitUpdated || itersWithoutAddedHPs > 5)
	{
		for (int k = processInfo->hyperplaneWaitingList.size(); k > 0; k--)
		{
			auto tmpItem = processInfo->hyperplaneWaitingList.at(k - 1);

			auto tmpIdx = tmpItem.first;
			std::vector<double> tmpPts;

			tmpPts = tmpItem.second;

			processInfo->MILPSolver->createHyperplane(tmpIdx, tmpPts);
		}

		processInfo->hyperplaneWaitingList.clear();
		itersWithoutAddedHPs = 0;
	}
	else
	{
		itersWithoutAddedHPs++;
	}
}

std::string TaskAddHyperplanes::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

