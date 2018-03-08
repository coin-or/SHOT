/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskExecuteRelaxationStrategy.h"

TaskExecuteRelaxationStrategy::TaskExecuteRelaxationStrategy(IMIPSolver *MIPSolver)
{
    this->MIPSolver = MIPSolver;

    relaxationStrategy = new RelaxationStrategyStandard(this->MIPSolver);

    ProcessInfo::getInstance().relaxationStrategy = relaxationStrategy;

    isInitialized = false;
}

TaskExecuteRelaxationStrategy::~TaskExecuteRelaxationStrategy()
{
    ProcessInfo::getInstance().relaxationStrategy = NULL;
    delete relaxationStrategy;
}

void TaskExecuteRelaxationStrategy::run()
{
    if (!isInitialized)
    {
        relaxationStrategy->setInitial();
        isInitialized = true;
    }
    else
    {
        relaxationStrategy->executeStrategy();
    }
}
std::string TaskExecuteRelaxationStrategy::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
