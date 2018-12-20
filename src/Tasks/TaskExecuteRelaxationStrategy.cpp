/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskExecuteRelaxationStrategy.h"

namespace SHOT
{

TaskExecuteRelaxationStrategy::TaskExecuteRelaxationStrategy(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->process->startTimer("DualStrategy");

    env->process->stopTimer("DualStrategy");
}

TaskExecuteRelaxationStrategy::~TaskExecuteRelaxationStrategy()
{
}

void TaskExecuteRelaxationStrategy::run()
{
    env->process->startTimer("DualStrategy");

    env->dualSolver->executeRelaxationStrategy();

    env->process->stopTimer("DualStrategy");
}
std::string TaskExecuteRelaxationStrategy::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT