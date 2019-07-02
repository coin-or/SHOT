/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskTerminate.h"

namespace SHOT
{

TaskTerminate::TaskTerminate(EnvironmentPtr envPtr) : TaskBase(envPtr) {}

TaskTerminate::~TaskTerminate() = default;

void TaskTerminate::run() {}

std::string TaskTerminate::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT