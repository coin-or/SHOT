/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MIPSolver/IRelaxationStrategy.h"
#include "../MIPSolver/RelaxationStrategyStandard.h"
#include "../MIPSolver/RelaxationStrategyNone.h"

namespace SHOT
{
class TaskExecuteRelaxationStrategy : public TaskBase
{
  public:
    TaskExecuteRelaxationStrategy(EnvironmentPtr envPtr);
    virtual ~TaskExecuteRelaxationStrategy();

    void run();
    virtual std::string getType();

  private:
    std::shared_ptr<IRelaxationStrategy> relaxationStrategy;

    bool isInitialized;
};
} // namespace SHOT