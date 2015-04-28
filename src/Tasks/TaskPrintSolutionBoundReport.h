#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

//#include "../OptProblems/OptProblemOriginal.h"

class TaskPrintSolutionBoundReport: public TaskBase
{
	public:
		TaskPrintSolutionBoundReport();
		virtual ~TaskPrintSolutionBoundReport();

		void run();
	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

		int itersSinceLastPrintout = 0;
		double timeLastPrintout = 0;
};
