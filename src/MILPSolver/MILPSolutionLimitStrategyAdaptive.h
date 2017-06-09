#pragma once
#include "IMILPSolutionLimitStrategy.h"

class MILPSolutionLimitStrategyAdaptive: public IMILPSolutionLimitStrategy
{
	public:
		MILPSolutionLimitStrategyAdaptive(IMILPSolver *MILPSolver);
		~MILPSolutionLimitStrategyAdaptive();

		virtual bool updateLimit();
		virtual int getNewLimit();
		virtual int getInitialLimit();

		int lastIterSolLimIncreased;
		int numSolLimIncremented;
};
