/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include <list>
#include <string>
#include <utility>

#include "Tasks/TaskBase.h"
#include "Tasks/TaskException.h"

namespace SHOT
{
class TaskBase;

class TaskHandler
{
public:
    TaskHandler(EnvironmentPtr envPtr);
    ~TaskHandler();

    void addTask(TaskBase* task, std::string taskID);
    bool getNextTask(TaskBase*& task);
    void setNextTask(std::string taskID);
    void clearTasks();

    TaskBase* getTask(std::string taskID);

    void terminate() { terminated = true; }
    inline bool isTerminated() { return terminated; }

private:
    std::list<std::pair<std::string, TaskBase*>>::iterator nextTask;
    std::string nextTaskID;
    std::list<std::pair<std::string, TaskBase*>> taskIDMap;
    std::list<TaskBase*> allTasks;

    EnvironmentPtr env;

    bool terminated = false;
};
}