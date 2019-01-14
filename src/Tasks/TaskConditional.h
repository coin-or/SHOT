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
class TaskConditional : public TaskBase
{
public:
    TaskConditional(
        EnvironmentPtr envPtr, std::function<bool()> conditionFunct, TaskBase* taskIfTrue, TaskBase* taskIfFalse);
    TaskConditional(EnvironmentPtr envPtr);
    virtual ~TaskConditional();

    void setTaskIfTrue(TaskBase* task);
    void setTaskIfFalse(TaskBase* task);
    void setCondition(std::function<bool()> conditionFunct);

    virtual void run();
    virtual std::string getType();

private:
    std::function<bool()> condition;

    TaskBase* taskIfTrue;
    TaskBase* taskIfFalse;
    bool taskFalseIsSet;

protected:
};
} // namespace SHOT