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

class TaskGoto : public TaskBase
{
  public:
    TaskGoto(EnvironmentPtr envPtr, std::string taskID);
    virtual ~TaskGoto();

    virtual void run();
    virtual std::string getType();

  private:
    std::string gotoTaskID;
};
