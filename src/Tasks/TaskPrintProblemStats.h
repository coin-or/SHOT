#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

#include "../OptProblems/OptProblemOriginal.h"

class TaskPrintProblemStats: public TaskBase
{
	public:
		TaskPrintProblemStats();
		virtual ~TaskPrintProblemStats();

		void run();

	private:
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
