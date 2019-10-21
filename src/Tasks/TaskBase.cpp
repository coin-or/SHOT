/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskBase.h"
#include "TaskException.h"

namespace SHOT
{

TaskBase::TaskBase(EnvironmentPtr envPtr) : env(envPtr) {}

bool TaskBase::isActive() { return (m_isActive); }

void TaskBase::activate() { m_isActive = true; }

void TaskBase::deactivate() { m_isActive = false; }

void TaskBase::initialize() {}

void TaskBase::run() {}

std::string TaskBase::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT