/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromSolutionPool.h"

TaskSelectPrimalCandidatesFromSolutionPool::TaskSelectPrimalCandidatesFromSolutionPool(EnvironmentPtr envPtr): TaskBase(envPtr)
{
}

TaskSelectPrimalCandidatesFromSolutionPool::~TaskSelectPrimalCandidatesFromSolutionPool()
{
}

void TaskSelectPrimalCandidatesFromSolutionPool::run()
{
	auto currIter = env->process->getCurrentIteration();

	if (currIter->isMIP())
	{
		env->process->startTimer("PrimalStrategy");
		auto allSolutions = env->process->getCurrentIteration()->solutionPoints;

		env->process->addPrimalSolutionCandidates(allSolutions, E_PrimalSolutionSource::MIPSolutionPool);

		env->process->stopTimer("PrimalStrategy");
	}
}

std::string TaskSelectPrimalCandidatesFromSolutionPool::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
