/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskGoto.h"

TaskGoto::TaskGoto(std::string taskID)
{
    gotoTaskID = taskID;
}

TaskGoto::~TaskGoto()
{
}

void TaskGoto::run()
{
    try
    {
        ProcessInfo::getInstance().tasks->setNextTask(gotoTaskID);
    }
    catch (TaskExceptionNotFound &e)
    {
        Output::getInstance().outputError("Could not find task: " + gotoTaskID);
    }
}

std::string TaskGoto::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
