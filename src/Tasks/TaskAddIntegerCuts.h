#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MIPSolver/IMIPSolver.h"

class TaskAddIntegerCuts: public TaskBase
{
	public:
		TaskAddIntegerCuts(IMIPSolver *MIPSolver);
		virtual ~TaskAddIntegerCuts();

		virtual void run();
		virtual std::string getType();
	private:

		IMIPSolver *MIPSolver;
};

