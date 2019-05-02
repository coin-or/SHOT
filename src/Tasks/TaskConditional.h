/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#include <functional>

namespace SHOT
{
class TaskConditional : public TaskBase
{
public:
    TaskConditional(
        EnvironmentPtr envPtr, std::function<bool()> conditionFunct, TaskPtr taskIfTrue, TaskPtr taskIfFalse);
    TaskConditional(EnvironmentPtr envPtr);
    ~TaskConditional() override;

    void setTaskIfTrue(TaskPtr task);
    void setTaskIfFalse(TaskPtr task);
    void setCondition(std::function<bool()> conditionFunct);

    void run() override;
    std::string getType() override;

private:
    std::function<bool()> condition;

    TaskPtr taskIfTrue;
    TaskPtr taskIfFalse;
    bool taskFalseIsSet;

protected:
};
} // namespace SHOT