/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromSolutionPool.h"

namespace SHOT
{

TaskSelectPrimalCandidatesFromSolutionPool::TaskSelectPrimalCandidatesFromSolutionPool(EnvironmentPtr envPtr)
    : TaskBase(envPtr)
{
}

TaskSelectPrimalCandidatesFromSolutionPool::~TaskSelectPrimalCandidatesFromSolutionPool() {}

void TaskSelectPrimalCandidatesFromSolutionPool::run()
{
    auto currIter = env->results->getCurrentIteration();

    if(currIter->isMIP() || env->results->usedSolutionStrategy == E_SolutionStrategy::NLP)
    {
        env->timing->startTimer("PrimalStrategy");
        auto allSolutions = env->results->getCurrentIteration()->solutionPoints;
        env->primalSolver->addPrimalSolutionCandidates(allSolutions, E_PrimalSolutionSource::MIPSolutionPool);

        env->timing->stopTimer("PrimalStrategy");
    }
}

std::string TaskSelectPrimalCandidatesFromSolutionPool::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT