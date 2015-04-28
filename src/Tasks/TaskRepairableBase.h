#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

class TaskRepairableBase: public TaskBase
{
	public:
		//TaskRepairableBase();
		~TaskRepairableBase();

		virtual void repair();

	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
