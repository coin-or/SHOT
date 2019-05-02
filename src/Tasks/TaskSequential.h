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
class TaskSequential : public TaskBase
{
public:
    TaskSequential(EnvironmentPtr envPtr);
    TaskSequential(EnvironmentPtr envPtr, int numberOfTasks);

    virtual ~TaskSequential();

    void addTasks(std::vector<TaskPtr> tasks);
    void addTask(TaskPtr task);

    virtual void run();
    virtual std::string getType();

private:
    std::vector<TaskPtr> m_tasks;
};
} // namespace SHOT