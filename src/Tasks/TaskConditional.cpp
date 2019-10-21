/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskConditional.h"

namespace SHOT
{

TaskConditional::TaskConditional(EnvironmentPtr envPtr) : TaskBase(envPtr) {}

TaskConditional::TaskConditional(
    EnvironmentPtr envPtr, std::function<bool()> conditionFunct, TaskPtr taskTrue, TaskPtr taskFalse)
    : TaskBase(envPtr)
{
    condition = conditionFunct;
    taskIfTrue = taskTrue;
    taskIfFalse = taskFalse;
    taskFalseIsSet = true;
}

TaskConditional::~TaskConditional() = default;

void TaskConditional::setCondition(std::function<bool()> conditionFunct) { condition = conditionFunct; }

void TaskConditional::setTaskIfTrue(TaskPtr task) { taskIfTrue = task; }

void TaskConditional::setTaskIfFalse(TaskPtr task)
{
    taskIfFalse = task;
    taskFalseIsSet = true;
}

void TaskConditional::run()
{
    bool tmpCondition = false;

    if(condition != nullptr)
    {
        tmpCondition = condition();
    }

    if(tmpCondition)
    {
        taskIfTrue->run();
    }
    else
    {
        if(taskFalseIsSet == true)
            taskIfFalse->run();
    }
}
std::string TaskConditional::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT