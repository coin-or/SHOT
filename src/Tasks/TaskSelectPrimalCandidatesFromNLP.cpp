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
    env->timing->startTimer("PrimalStrategy");
    env->timing->startTimer("PrimalBoundStrategyNLP");

    primalStrategyFixedNLP = std::make_unique<PrimalSolutionStrategyFixedNLP>(env);

    env->timing->stopTimer("PrimalBoundStrategyNLP");
    env->timing->stopTimer("PrimalStrategy");
}

TaskSelectPrimalCandidatesFromNLP::~TaskSelectPrimalCandidatesFromNLP() {}

void TaskSelectPrimalCandidatesFromNLP::run()
{
    auto currIter = env->results->getCurrentIteration();

    if(currIter->isMIP() && env->results->getRelativeObjectiveGap() > 1e-10)
    {
        env->timing->startTimer("PrimalStrategy");
        env->timing->startTimer("PrimalBoundStrategyNLP");

        primalStrategyFixedNLP->runStrategy();

        env->timing->stopTimer("PrimalBoundStrategyNLP");
        env->timing->stopTimer("PrimalStrategy");
    }
}

std::string TaskSelectPrimalCandidatesFromNLP::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT