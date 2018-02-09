#pragma once

#include "TaskBase.h"
#include "../ProcessInfo.h"
#include "../MIPSolver/IMIPSolver.h"

class TaskSelectPrimalFixedNLPPointsFromSolutionPool: public TaskBase
{
	public:
		TaskSelectPrimalFixedNLPPointsFromSolutionPool();
		virtual ~TaskSelectPrimalFixedNLPPointsFromSolutionPool();

		virtual void run();
		virtual std::string getType();
	private:

};
