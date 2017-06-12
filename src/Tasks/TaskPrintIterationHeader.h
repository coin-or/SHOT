#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../OptProblems/OptProblemOriginal.h"

class TaskPrintIterationHeader: public TaskBase
{
	public:
		TaskPrintIterationHeader();
		virtual ~TaskPrintIterationHeader();

		void run();
		virtual std::string getType();

	private:
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
