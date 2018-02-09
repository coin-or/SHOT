/*
 * TaskAddHyperplanes.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#include "TaskAddHyperplanes.h"

TaskAddHyperplanes::TaskAddHyperplanes(IMIPSolver *MIPSolver)
{

	itersWithoutAddedHPs = 0;

	this->MIPSolver = MIPSolver;
}

TaskAddHyperplanes::~TaskAddHyperplanes()
{
	// TODO Auto-generated destructor stub
}

void TaskAddHyperplanes::run()
{
	this->MIPSolver = MIPSolver;
	auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

	if (!currIter->isMIP() || !Settings::getInstance().getBoolSetting("HyperplaneCuts.Delay", "Dual")
			|| !currIter->MIPSolutionLimitUpdated || itersWithoutAddedHPs > 5)
	{
		int addedHyperplanes = 0;

		for (int k = ProcessInfo::getInstance().hyperplaneWaitingList.size(); k > 0; k--)
		{
			if (addedHyperplanes >= Settings::getInstance().getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual")) break;

			auto tmpItem = ProcessInfo::getInstance().hyperplaneWaitingList.at(k - 1);

			if (tmpItem.source == E_HyperplaneSource::PrimalSolutionSearchInteriorObjective)
			{
				MIPSolver->createInteriorHyperplane(tmpItem);
			}
			else
			{
				MIPSolver->createHyperplane(tmpItem);

				ProcessInfo::getInstance().addedHyperplanes.push_back(tmpItem);

				addedHyperplanes++;
			}

		}

		ProcessInfo::getInstance().hyperplaneWaitingList.clear();
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

