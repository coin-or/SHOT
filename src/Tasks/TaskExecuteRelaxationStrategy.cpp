/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskExecuteRelaxationStrategy.h"

#include "../DualSolver.h"
#include "../Timing.h"

#include "../MIPSolver/IMIPSolver.h"

namespace SHOT
{

TaskExecuteRelaxationStrategy::TaskExecuteRelaxationStrategy(EnvironmentPtr envPtr) : TaskBase(envPtr) {}

TaskExecuteRelaxationStrategy::~TaskExecuteRelaxationStrategy() = default;

void TaskExecuteRelaxationStrategy::run()
{
    env->timing->startTimer("DualStrategy");

    env->dualSolver->MIPSolver->executeRelaxationStrategy();

    env->timing->stopTimer("DualStrategy");
}
std::string TaskExecuteRelaxationStrategy::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT