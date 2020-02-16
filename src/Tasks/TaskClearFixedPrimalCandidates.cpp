/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskClearFixedPrimalCandidates.h"

#include "../PrimalSolver.h"
#include "../Timing.h"

namespace SHOT
{

TaskClearFixedPrimalCandidates::TaskClearFixedPrimalCandidates(EnvironmentPtr envPtr) : TaskBase(envPtr) {}

TaskClearFixedPrimalCandidates::~TaskClearFixedPrimalCandidates() = default;

void TaskClearFixedPrimalCandidates::run()
{
    env->timing->startTimer("PrimalStrategy");
    env->timing->startTimer("PrimalBoundStrategyNLP");

    env->primalSolver->fixedPrimalNLPCandidates.clear();

    env->timing->stopTimer("PrimalBoundStrategyNLP");
    env->timing->stopTimer("DualStrategy");
}

std::string TaskClearFixedPrimalCandidates::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT