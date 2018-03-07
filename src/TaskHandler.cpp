/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskHandler.h"

TaskHandler::TaskHandler()
{
    nextTask = taskIDMap.begin();
}

TaskHandler::~TaskHandler()
{
    for (auto task : taskIDMap)
    {
        delete task.second;
    }
}

void TaskHandler::addTask(TaskBase *task, std::string taskID)
{
    taskIDMap.push_back(std::make_pair(taskID, task));

    if (nextTask == taskIDMap.end())
    {
        nextTask = taskIDMap.begin();
    }
}

bool TaskHandler::getNextTask(TaskBase *&task)
{
    if (nextTask == taskIDMap.end())
        return (false);

    task = (nextTask->second);
    nextTask++;

    return (true);
}

void TaskHandler::setNextTask(std::string taskID)
{
    bool isFound = false;

    for (std::list<std::pair<std::string, TaskBase *>>::iterator it = taskIDMap.begin(); it != taskIDMap.end(); ++it)
    {
        if (it->first == taskID)
        {
            nextTask = it;
            isFound = true;
            break;
        }
    }

    if (!isFound)
    {
        // Cannot find the specified task
        TaskExceptionNotFound e(taskID);
        throw(e);
    }
}

void TaskHandler::clearTasks()
{
    taskIDMap.clear();
    nextTask == taskIDMap.end();
}

TaskBase *TaskHandler::getTask(std::string taskID)
{
    for (std::list<std::pair<std::string, TaskBase *>>::iterator it = taskIDMap.begin(); it != taskIDMap.end(); ++it)
    {
        if (it->first == taskID)
        {
            return (it->second);
        }
    }

    // Cannot find the specified task
    TaskExceptionNotFound e(taskID);
    throw(e);
}
