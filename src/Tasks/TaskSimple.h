/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

namespace SHOT
{
class TaskSimple : public TaskBase
{
public:
    TaskSimple(EnvironmentPtr envPtr, std::function<bool()> taskFunction);
    TaskSimple(EnvironmentPtr envPtr);
    virtual ~TaskSimple();

    void setFunction(std::function<bool()> taskFunction);

    virtual void run();
    virtual std::string getType();

private:
    std::function<bool()> task;
};
} // namespace SHOT