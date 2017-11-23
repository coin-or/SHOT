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
		//std::cout << "Next task is of type: " << T->getType() << std::endl;
		T->run();
		//std::cout << "Finished task" << std::endl;
	}
}

void TaskSequential::addTasks(std::vector<TaskBase*> tasks)
{
	for (auto T : tasks)
		addTask(T);
}

void TaskSequential::addTask(TaskBase* task)
{
	m_tasks.emplace_back(task);
}

std::string TaskSequential::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
