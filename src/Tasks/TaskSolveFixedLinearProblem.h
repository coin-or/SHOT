#pragma once

#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MIPSolver/IMIPSolver.h"

class TaskSolveFixedLinearProblem: public TaskBase
{
	public:
		TaskSolveFixedLinearProblem(IMIPSolver *MIPSolver);
		virtual ~TaskSolveFixedLinearProblem();
		virtual void run();
		virtual std::string getType();
	private:

		std::vector<int> discreteVariableIndexes;
		std::vector<std::vector<double>> testedPoints;

		std::vector<double> lastSolution;
		double lastPrimalBound;
		IMIPSolver *MIPSolver;
};

