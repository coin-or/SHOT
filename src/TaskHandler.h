#pragma once
#include "vector"
#include "list"
#include "unordered_map"
#include "algorithm"
#include "Tasks/TaskBase.h"
#include "Tasks/TaskException.h"

class TaskHandler
{
	public:
		TaskHandler();
		~TaskHandler();

		void addTask(TaskBase *task, std::string taskID);
		bool getNextTask(TaskBase * &task);
		void setNextTask(std::string taskID);
		void clearTasks();

	private:
		//std::list<TaskBase*> allTasks;
		std::list<std::pair<std::string, TaskBase*>>::iterator nextTask;
		std::string nextTaskID;
		//bool terminateNext;
		std::list<std::pair<std::string, TaskBase*>> taskIDMap;
};
