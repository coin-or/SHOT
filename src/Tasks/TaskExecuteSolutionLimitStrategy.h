#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MIPSolver/IMIPSolutionLimitStrategy.h"
#include "../MIPSolver/MIPSolutionLimitStrategyUnlimited.h"
#include "../MIPSolver/MIPSolutionLimitStrategyIncrease.h"
#include "../MIPSolver/MIPSolutionLimitStrategyAdaptive.h"

class TaskExecuteSolutionLimitStrategy: public TaskBase
{
	public:
		TaskExecuteSolutionLimitStrategy(IMIPSolver *MIPSolver);
		~TaskExecuteSolutionLimitStrategy();

		void run();
		virtual std::string getType();

	private:

		IMIPSolutionLimitStrategy *solutionLimitStrategy;

		IMIPSolver *MIPSolver;

		bool isInitialized;
		bool temporaryOptLimitUsed;
		int previousSolLimit;
};

