#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"
#include <vector>

class TaskSequential: public TaskBase
{
	public:
		TaskSequential();
		TaskSequential(int numberOfTasks);

		~TaskSequential();

		void addTasks(std::vector<TaskBase*> tasks);
		void addTask(TaskBase* task);

		virtual void run();
		virtual std::string getType();

	private:
		std::vector<TaskBase*> m_tasks;

};
