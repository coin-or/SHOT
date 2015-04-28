#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

#include "../OptProblems/OptProblemOriginal.h"

class TaskPrintIterationReport: public TaskBase
{
	public:
		TaskPrintIterationReport();
		virtual ~TaskPrintIterationReport();

		void run();
	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
