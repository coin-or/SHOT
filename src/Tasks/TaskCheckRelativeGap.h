/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

namespace SHOT
{
class TaskCheckRelativeGap : public TaskBase
{
  public:
    TaskCheckRelativeGap(EnvironmentPtr envPtr, std::string taskIDTrue);
    virtual ~TaskCheckRelativeGap();

    virtual void run();
    virtual std::string getType();

  private:
    std::string taskIDIfTrue;
};
} // namespace SHOT