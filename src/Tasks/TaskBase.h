/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "../Environment.h"

#include <string>

namespace SHOT
{

class TaskBase
{
public:
    // A deactivated task is skipped by the task handler and by TaskSequential
    bool isActive() const { return active; }
    void activate() { active = true; }
    void deactivate() { active = false; }

    virtual void initialize();

    virtual std::string getType();

    virtual void run();

    TaskBase(EnvironmentPtr envPtr);
    virtual ~TaskBase() = default;

protected:
    EnvironmentPtr env;

private:
    bool active = true;
};

using TaskPtr =std::shared_ptr<TaskBase> ;

} // namespace SHOT