/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSequential.h"

TaskSequential::TaskSequential()
{
}

TaskSequential::TaskSequential(int numberOfTasks)
{

	m_tasks.reserve(numberOfTasks);
}

TaskSequential::~TaskSequential()
{
	for (auto T : m_tasks)
		delete (T);
}

void TaskSequential::run()
{
	for (auto T : m_tasks)
	{
		T->run();
	}
}

void TaskSequential::addTasks(std::vector<TaskBase *> tasks)
{
	for (auto T : tasks)
		addTask(T);
}

void TaskSequential::addTask(TaskBase *task)
{
	m_tasks.emplace_back(task);
}

std::string TaskSequential::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
