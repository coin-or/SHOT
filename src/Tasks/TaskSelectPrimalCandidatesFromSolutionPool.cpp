/*
 * TaskSelectPrimalCandidatesFromSolutionPool.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include <TaskSelectPrimalCandidatesFromSolutionPool.h>

TaskSelectPrimalCandidatesFromSolutionPool::TaskSelectPrimalCandidatesFromSolutionPool()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskSelectPrimalCandidatesFromSolutionPool::~TaskSelectPrimalCandidatesFromSolutionPool()
{
	// TODO Auto-generated destructor stub
}

void TaskSelectPrimalCandidatesFromSolutionPool::run()
{
	auto currIter = processInfo->getCurrentIteration();

	/*if (processInfo->originalProblem->isObjectiveFunctionNonlinear())
	 {
	 for (int i = 0; i < currIter->solutionPoints.size(); i++)
	 {
	 auto dualSol = currIter->solutionPoints.at(i);

	 if (dualSol.maxDeviation.value <= 0) continue;

	 auto tmpDualSolPoint = dualSol.point;

	 double mu = dualSol.objectiveValue;
	 double error = processInfo->originalProblem->getProblemInstance()->calculateFunctionValue(-1,
	 &tmpDualSolPoint.at(0), true) - mu;

	 vector<double> tmpPoint(dualSol.point);
	 tmpPoint.back() = error;

//	 std::cout << "Added " << tmpPoint.back() << std::endl;

	 processInfo->addPrimalSolutionCandidate(tmpPoint, E_PrimalSolutionSource::ObjectiveConstraint,
	 currIter->iterationNumber);

	 //std::cout << "adding primal solution candidate" << std::endl;
	 }

	 }*/

	if (currIter->isMILP() && processInfo->getRelativeObjectiveGap() > 1e-10)
	{
		processInfo->startTimer("PrimalBoundTotal");
		auto allSolutions = processInfo->getCurrentIteration()->solutionPoints;
		processInfo->addPrimalSolutionCandidates(allSolutions, E_PrimalSolutionSource::MILPSolutionPool);

		processInfo->stopTimer("PrimalBoundTotal");
	}

}

std::string TaskSelectPrimalCandidatesFromSolutionPool::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

