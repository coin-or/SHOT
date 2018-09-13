/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskExecuteRelaxationStrategy.h"

TaskExecuteRelaxationStrategy::TaskExecuteRelaxationStrategy(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->process->startTimer("DualStrategy");

    relaxationStrategy = new RelaxationStrategyStandard(env);

    env->process->relaxationStrategy = relaxationStrategy;

    isInitialized = false;

    env->process->stopTimer("DualStrategy");
}

TaskExecuteRelaxationStrategy::~TaskExecuteRelaxationStrategy()
{
    env->process->relaxationStrategy = NULL;
    delete relaxationStrategy;
}

void TaskExecuteRelaxationStrategy::run()
{
    env->process->startTimer("DualStrategy");
    if (!isInitialized)
    {
        relaxationStrategy->setInitial();
        isInitialized = true;
    }
    else
    {
        relaxationStrategy->executeStrategy();
    }

    env->process->stopTimer("DualStrategy");
}
std::string TaskExecuteRelaxationStrategy::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
