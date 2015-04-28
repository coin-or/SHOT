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
	//processInfo = ProcessInfo::getInstance();
	//settings = SHOTSettings::Settings::getInstance();
}

void TaskBase::run()
{
	std::cout << "What?!" << std::endl;
}
