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
		virtual std::string getType();

	private:
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
