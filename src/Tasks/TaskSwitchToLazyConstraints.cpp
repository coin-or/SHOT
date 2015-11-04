/*
 * TaskSwitchToLazyConstraints.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include <TaskSwitchToLazyConstraints.h>

TaskSwitchToLazyConstraints::TaskSwitchToLazyConstraints()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

}

TaskSwitchToLazyConstraints::~TaskSwitchToLazyConstraints()
{
// TODO Auto-generated destructor stub
}

void TaskSwitchToLazyConstraints::run()
{

	auto genHyperplanes = processInfo->MILPSolver->getGeneratedHyperplanes();
	auto currIter = processInfo->getCurrentIteration();

	if (!currIter->isMILP()) return;

	processInfo->startTimer("LazyChange");

	for (int i = 0; i < genHyperplanes->size(); i++)
	{
		//auto currHyperplane = genHyperplanes->at(i);

		if (genHyperplanes->at(i).isLazy) continue;
		if (genHyperplanes->at(i).isRemoved) continue;
		if (genHyperplanes->at(i).source == E_HyperplaneSource::MIPOptimalLinesearch
				&& currIter->iterationNumber - genHyperplanes->at(i).generatedIter < 30) continue;
		/*if (genHyperplanes->at(i).source == E_HyperplaneSource::LPFixedIntegers
		 || genHyperplanes->at(i).source == E_HyperplaneSource::LPRelaxedLinesearch
		 || genHyperplanes->at(i).source == E_HyperplaneSource::LPRelaxedSolutionPoint
		 || genHyperplanes->at(i).source == E_HyperplaneSource::MIPSolutionPoolLinesearch)
		 {
		 //std::cout << "not changed: " << genHyperplanes->at(i).isLazy << std::endl;
		 processInfo->MILPSolver->changeConstraintToLazy(genHyperplanes->at(i));
		 //std::cout << "is changed: " << genHyperplanes->at(i).isLazy << std::endl;
		 }*/

		if (currIter->iterationNumber - genHyperplanes->at(i).generatedIter > 30) processInfo->MILPSolver->changeConstraintToLazy(
				genHyperplanes->at(i));

		if ((genHyperplanes->at(i).source == E_HyperplaneSource::LPFixedIntegers
				|| genHyperplanes->at(i).source == E_HyperplaneSource::LPRelaxedLinesearch
				|| genHyperplanes->at(i).source == E_HyperplaneSource::LPRelaxedSolutionPoint)
				&& currIter->iterationNumber - genHyperplanes->at(i).generatedIter > 1) processInfo->MILPSolver->changeConstraintToLazy(
				genHyperplanes->at(i));

	}
	processInfo->stopTimer("LazyChange");

}
std::string TaskSwitchToLazyConstraints::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

