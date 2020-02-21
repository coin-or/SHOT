/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSequential.h"

#include <vector>

#include "../Output.h"

namespace SHOT
{

TaskSequential::TaskSequential(EnvironmentPtr envPtr) : TaskBase(envPtr) {}

TaskSequential::TaskSequential(EnvironmentPtr envPtr, int numberOfTasks) : TaskBase(envPtr)
{
    m_tasks.reserve(numberOfTasks);
}

TaskSequential::~TaskSequential() = default;

void TaskSequential::run()
{
    for(auto& T : m_tasks)
    {
#ifdef SIMPLE_OUTPUT_CHARS
        env->output->outputTrace("---- Started task:  " + T->getType());
        T->run();
        env->output->outputTrace("---- Finished task: " + T->getType());
#else
        env->output->outputTrace("┌─── Started task:  " + T->getType());
        T->run();
        env->output->outputTrace("└─── Finished task: " + T->getType());
#endif
    }
}

void TaskSequential::addTasks(std::vector<TaskPtr> tasks)
{
    for(auto& T : tasks)
        addTask(T);
}

void TaskSequential::addTask(TaskPtr task) { m_tasks.emplace_back(task); }

std::string TaskSequential::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT