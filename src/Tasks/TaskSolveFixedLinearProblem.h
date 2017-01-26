#pragma once

#include <TaskBase.h>
#include "../ProcessInfo.h"

#include "../LinesearchMethod/ILinesearchMethod.h"
#include "../LinesearchMethod/LinesearchMethodBisection.h"
#include "../LinesearchMethod/LinesearchMethodBoost.h"
#include "../MILPSolver/IMILPSolver.h"

#include <boost/math/tools/roots.hpp>

class TaskSolveFixedLinearProblem: public TaskBase
{
	public:
		TaskSolveFixedLinearProblem();
		virtual ~TaskSolveFixedLinearProblem();
		virtual void run();
		virtual std::string getType();
	private:
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
		ILinesearchMethod *linesearchMethod;

		std::vector<int> discreteVariableIndexes;
		std::vector<std::vector<double>> testedPoints;

		std::vector<double> lastSolution;
};

