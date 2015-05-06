#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MILPSolver/IMILPSolutionLimitStrategy.h"
#include "../MILPSolver/MILPSolutionLimitStrategyUnlimited.h"
#include "../MILPSolver/MILPSolutionLimitStrategyIncrease.h"
#include "../MILPSolver/MILPSolutionLimitStrategyAdaptive.h"

#include "../ConstraintToleranceUpdateStrategy/IConstraintToleranceUpdateStrategy.h"
#include "../ConstraintToleranceUpdateStrategy/ConstraintToleranceUpdateStrategyDecreasing.h"

class TaskExecuteSolutionLimitStrategy: public TaskBase
{
	public:
		TaskExecuteSolutionLimitStrategy();
		~TaskExecuteSolutionLimitStrategy();

		void run();
		virtual std::string getType();

	private:

		IMILPSolutionLimitStrategy *solutionLimitStrategy;
		IConstraintToleranceUpdateStrategy *constrTolUpdateStrategy;

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

		bool isInitialized;
		bool temporaryOptLimitUsed;
		int previousSolLimit;
};

