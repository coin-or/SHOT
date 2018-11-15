/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromNLP.h"

namespace SHOT
{

TaskSelectPrimalCandidatesFromNLP::TaskSelectPrimalCandidatesFromNLP(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->process->startTimer("PrimalStrategy");
    env->process->startTimer("PrimalBoundStrategyNLP");

    primalStrategyFixedNLP = new PrimalSolutionStrategyFixedNLP(env);

    env->process->stopTimer("PrimalBoundStrategyNLP");
    env->process->stopTimer("PrimalStrategy");
}

TaskSelectPrimalCandidatesFromNLP::~TaskSelectPrimalCandidatesFromNLP()
{
    delete primalStrategyFixedNLP;
}

void TaskSelectPrimalCandidatesFromNLP::run()
{
    auto currIter = env->process->getCurrentIteration();

    if (currIter->isMIP() && env->process->getRelativeObjectiveGap() > 1e-10)
    {
        env->process->startTimer("PrimalStrategy");
        env->process->startTimer("PrimalBoundStrategyNLP");

        primalStrategyFixedNLP->runStrategy();

        env->process->stopTimer("PrimalBoundStrategyNLP");
        env->process->stopTimer("PrimalStrategy");
    }
}

std::string TaskSelectPrimalCandidatesFromNLP::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT