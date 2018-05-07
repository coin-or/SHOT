/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskInitializeIteration.h"

TaskInitializeIteration::TaskInitializeIteration()
{
}

TaskInitializeIteration::~TaskInitializeIteration()
{
}

void TaskInitializeIteration::run()
{
    ProcessInfo::getInstance().createIteration();
}

std::string TaskInitializeIteration::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
