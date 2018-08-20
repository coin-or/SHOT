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
#include "../OptProblems/OptProblemOriginal.h"

namespace SHOT
{
class TaskUpdateNonlinearObjectiveByLinesearch : public TaskBase
{
  public:
    TaskUpdateNonlinearObjectiveByLinesearch(EnvironmentPtr envPtr);
    virtual ~TaskUpdateNonlinearObjectiveByLinesearch();
    virtual void run();
    virtual std::string getType();

    bool updateObjectiveInPoint(SolutionPoint &solution);

  private:
};
} // namespace SHOT