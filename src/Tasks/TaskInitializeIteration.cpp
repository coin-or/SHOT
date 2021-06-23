/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskInitializeIteration.h"

#include "../DualSolver.h"
#include "../Results.h"

#include "../MIPSolver/IMIPSolver.h"

namespace SHOT
{

TaskInitializeIteration::TaskInitializeIteration(EnvironmentPtr envPtr) : TaskBase(envPtr) { }

TaskInitializeIteration::~TaskInitializeIteration() = default;

void TaskInitializeIteration::run()
{
    env->results->createIteration();
    env->results->getCurrentIteration()->isDualProblemDiscrete
        = env->dualSolver->MIPSolver->getDiscreteVariableStatus();
    env->results->getCurrentIteration()->dualProblemClass = env->dualSolver->MIPSolver->getProblemClass();
}

std::string TaskInitializeIteration::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT