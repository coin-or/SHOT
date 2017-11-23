#pragma once

#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MILPSolver/IMILPSolver.h"

class TaskSolveFixedLinearProblem: public TaskBase
{
	public:
		TaskSolveFixedLinearProblem(IMILPSolver *MILPSolver);
		virtual ~TaskSolveFixedLinearProblem();
		virtual void run();
		virtual std::string getType();
	private:

		std::vector<int> discreteVariableIndexes;
		std::vector<std::vector<double>> testedPoints;

		std::vector<double> lastSolution;
		double lastPrimalBound;
		IMILPSolver *MILPSolver;
};

