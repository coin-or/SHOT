#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"
#include "../MILPSolver/IMILPSolver.h"
class TaskPrintSolution: public TaskBase
{
	public:
		TaskPrintSolution();
		~TaskPrintSolution();

		virtual void run();

	private:
		SHOTSettings::Settings * settings;
		ProcessInfo *processInfo;
};

