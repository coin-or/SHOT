#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

#include "../MILPSolver/IMILPSolver.h"

class TaskAddIntegerCuts: public TaskBase
{
	public:
		TaskAddIntegerCuts(IMILPSolver *MILPSolver);
		virtual ~TaskAddIntegerCuts();

		virtual void run();
		virtual std::string getType();
	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
		IMILPSolver *MILPSolver;
};

