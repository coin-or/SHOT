/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSimple.h"
#include "TaskException.h"

namespace SHOT
{

TaskSimple::TaskSimple(EnvironmentPtr envPtr) : TaskBase(envPtr) {}

TaskSimple::TaskSimple(EnvironmentPtr envPtr, std::function<bool()> taskFunction) : TaskBase(envPtr)
{
    task = taskFunction;
}

TaskSimple::~TaskSimple() = default;

void TaskSimple::setFunction(std::function<bool()> taskFunction) { task = taskFunction; }

void TaskSimple::run()
{
    if(task != nullptr)
    {
        task();
    }
    else
    {
        TaskExceptionFunctionNotDefined e(env, "TaskSimple");

        throw(e);
    }
}

std::string TaskSimple::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT