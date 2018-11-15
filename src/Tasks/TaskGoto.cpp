/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskGoto.h"
namespace SHOT
{

TaskGoto::TaskGoto(EnvironmentPtr envPtr, std::string taskID) : TaskBase(envPtr), gotoTaskID(taskID)
{
}

TaskGoto::~TaskGoto()
{
}

void TaskGoto::run()
{
    try
    {
        env->tasks->setNextTask(gotoTaskID);
    }
    catch (TaskExceptionNotFound &e)
    {
        env->output->outputError("Could not find task: " + gotoTaskID);
    }
}

std::string TaskGoto::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT