#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

//#include "../OptProblems/OptProblemOriginal.h"

class TaskPrintSolutionBoundReport: public TaskBase
{
	public:
		TaskPrintSolutionBoundReport();
		virtual ~TaskPrintSolutionBoundReport();

		void run();
		virtual std::string getType();
	private:

		int itersSinceLastPrintout;
		double timeLastPrintout;
};
