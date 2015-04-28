#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"
#include "UtilityFunctions.h"

class TaskInitializeIteration: public TaskBase
{
	public:
		TaskInitializeIteration();
		~TaskInitializeIteration();

		void run();

	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};

