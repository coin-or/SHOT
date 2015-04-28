#pragma once
#include "Enums.h"
#include <iostream>
#include "Tasks/TaskException.h"

class TaskBase
{
	public:
		virtual bool isActive();
		virtual void activate();
		virtual void deactivate();

		virtual void initialize();

		virtual void run();

	protected:

	private:

		bool m_isActive;
};
