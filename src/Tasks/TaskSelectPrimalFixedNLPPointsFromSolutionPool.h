#pragma once

#include "TaskBase.h"
#include "../ProcessInfo.h"
#include "../MILPSolver/IMILPSolver.h"

class TaskSelectPrimalFixedNLPPointsFromSolutionPool: public TaskBase
{
	public:
		TaskSelectPrimalFixedNLPPointsFromSolutionPool();
		virtual ~TaskSelectPrimalFixedNLPPointsFromSolutionPool();

		virtual void run();
		virtual std::string getType();
	private:

};
