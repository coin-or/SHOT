#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MIPSolver/IMIPSolver.h"

class TaskPresolve: public TaskBase
{
	public:
		TaskPresolve(IMIPSolver *MIPSolver);
		virtual ~TaskPresolve();

		virtual void run();
		virtual std::string getType();
	private:

		bool isPresolved;
		IMIPSolver *MIPSolver;
};

