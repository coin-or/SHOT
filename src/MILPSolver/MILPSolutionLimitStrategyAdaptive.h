#pragma once
#include "IMILPSolutionLimitStrategy.h"

class MILPSolutionLimitStrategyAdaptive: public IMILPSolutionLimitStrategy
{
	public:
		MILPSolutionLimitStrategyAdaptive();
		~MILPSolutionLimitStrategyAdaptive();

		virtual bool updateLimit();
		virtual int getNewLimit();
		virtual int getInitialLimit();

		int lastIterSolLimIncreased;
		int numSolLimIncremented;
};
