#pragma once
#include "IMILPSolutionLimitStrategy.h"

class MILPSolutionLimitStrategyUnlimited: public IMILPSolutionLimitStrategy
{
	public:
		MILPSolutionLimitStrategyUnlimited();
		~MILPSolutionLimitStrategyUnlimited();

		virtual bool updateLimit();
		virtual int getNewLimit();
		virtual int getInitialLimit();
};
