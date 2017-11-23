#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MILPSolver/IMILPSolver.h"

class TaskPresolve: public TaskBase
{
	public:
		TaskPresolve(IMILPSolver *MILPSolver);
		virtual ~TaskPresolve();

		virtual void run();
		virtual std::string getType();
	private:

		bool isPresolved;
		IMILPSolver *MILPSolver;
};

