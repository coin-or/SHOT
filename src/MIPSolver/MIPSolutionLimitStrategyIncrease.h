#pragma once
#include "IMIPSolutionLimitStrategy.h"

class MIPSolutionLimitStrategyIncrease: public IMIPSolutionLimitStrategy
{
	public:
		MIPSolutionLimitStrategyIncrease(IMIPSolver *MIPSolver);
		~MIPSolutionLimitStrategyIncrease();

		virtual bool updateLimit();
		virtual int getNewLimit();
		virtual int getInitialLimit();

		int lastIterSolLimIncreased;
		int numSolLimIncremented;
		int lastIterOptimal;
};
