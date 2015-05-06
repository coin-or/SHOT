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
		virtual std::string getType();

	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};

