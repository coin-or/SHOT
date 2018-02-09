#pragma once
#include "IMIPSolutionLimitStrategy.h"

class MIPSolutionLimitStrategyAdaptive: public IMIPSolutionLimitStrategy
{
	public:
		MIPSolutionLimitStrategyAdaptive(IMIPSolver *MIPSolver);
		~MIPSolutionLimitStrategyAdaptive();

		virtual bool updateLimit();
		virtual int getNewLimit();
		virtual int getInitialLimit();

		int lastIterSolLimIncreased;
		int numSolLimIncremented;
};
