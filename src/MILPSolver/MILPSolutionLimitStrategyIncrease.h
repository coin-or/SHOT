#pragma once
#include "IMILPSolutionLimitStrategy.h"

class MILPSolutionLimitStrategyIncrease: public IMILPSolutionLimitStrategy
{
	public:
		MILPSolutionLimitStrategyIncrease(IMILPSolver *MILPSolver);
		~MILPSolutionLimitStrategyIncrease();

		virtual bool updateLimit();
		virtual int getNewLimit();
		virtual int getInitialLimit();

		int lastIterSolLimIncreased;
		int numSolLimIncremented;
		int lastIterOptimal;
};
