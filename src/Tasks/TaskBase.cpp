#include "TaskBase.h"

bool TaskBase::isActive()
{
	return (m_isActive);
}

void TaskBase::activate()
{
	m_isActive = true;
}

void TaskBase::deactivate()
{
	m_isActive = false;
}

void TaskBase::initialize()
{
}

void TaskBase::run()
{
	std::cout << "What?!" << std::endl;
}

std::string TaskBase::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
