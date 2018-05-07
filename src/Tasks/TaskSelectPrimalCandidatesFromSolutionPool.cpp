/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromSolutionPool.h"

TaskSelectPrimalCandidatesFromSolutionPool::TaskSelectPrimalCandidatesFromSolutionPool()
{
}

TaskSelectPrimalCandidatesFromSolutionPool::~TaskSelectPrimalCandidatesFromSolutionPool()
{
}

void TaskSelectPrimalCandidatesFromSolutionPool::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->isMIP())
	{
		ProcessInfo::getInstance().startTimer("PrimalStrategy");
		auto allSolutions = ProcessInfo::getInstance().getCurrentIteration()->solutionPoints;

		ProcessInfo::getInstance().addPrimalSolutionCandidates(allSolutions, E_PrimalSolutionSource::MIPSolutionPool);

		ProcessInfo::getInstance().stopTimer("PrimalStrategy");
	}
}

std::string TaskSelectPrimalCandidatesFromSolutionPool::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
