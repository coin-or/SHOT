/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskHandler.h"

#include <algorithm>

namespace SHOT
{

TaskHandler::TaskHandler(EnvironmentPtr envPtr)
{
    env = envPtr;

    nextTask = taskIDMap.begin();
}

void TaskHandler::addTask(TaskPtr task, std::string taskID)
{
    taskIDMap.emplace_back(taskID, task);

    if(nextTask == taskIDMap.end())
    {
        nextTask = taskIDMap.begin();
    }

    // Checks if this task has been added previously, otherwise adds it to the list of all tasks
    bool found = (std::find(allTasks.begin(), allTasks.end(), task) != allTasks.end());

    if(!found)
        allTasks.push_back(task);
}

bool TaskHandler::getNextTask(TaskPtr& task)
{
    if(nextTask == taskIDMap.end())
        return (false);

    task = (nextTask->second);
    nextTask++;

    return (true);
}

void TaskHandler::setNextTask(std::string taskID)
{
    bool isFound = false;

    for(auto it = taskIDMap.begin(); it != taskIDMap.end(); ++it)
    {
        if(it->first == taskID)
        {
            nextTask = it;
            isFound = true;
            break;
        }
    }

    if(!isFound)
    {
        // Cannot find the specified task
        TaskExceptionNotFound e(env, taskID);
        throw(e);
    }
}

void TaskHandler::clearTasks()
{
    taskIDMap.clear();
    nextTask = taskIDMap.end();
}

TaskPtr TaskHandler::getTask(std::string taskID)
{
    for(auto & it : taskIDMap)
    {
        if(it.first == taskID)
        {
            return (it.second);
        }
    }

    // Cannot find the specified task
    TaskExceptionNotFound e(env, taskID);
    throw(e);
}
} // namespace SHOT