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
#include "../LinesearchMethod/LinesearchMethodBoost.h"
#include "../LinesearchMethod/LinesearchMethodBisection.h"

namespace SHOT
{
class TaskInitializeLinesearch : public TaskBase
{
  public:
    TaskInitializeLinesearch(EnvironmentPtr envPtr);
    virtual ~TaskInitializeLinesearch();

    virtual void run();

    virtual std::string getType();
};
} // namespace SHOT