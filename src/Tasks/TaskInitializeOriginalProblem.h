/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#include "../MIPSolver/IMIPSolver.h"

namespace SHOT
{
class TaskInitializeOriginalProblem : public TaskBase
{
  public:
    TaskInitializeOriginalProblem(EnvironmentPtr envPtr, OSInstance *originalInstance);
    virtual ~TaskInitializeOriginalProblem();

    void run();
    virtual std::string getType();

  private:
    OptProblem *problem;
    OSInstance *instance;
};
} // namespace SHOT