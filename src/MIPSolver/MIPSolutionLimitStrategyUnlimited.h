#pragma once
#include "IMIPSolutionLimitStrategy.h"

class MIPSolutionLimitStrategyUnlimited: public IMIPSolutionLimitStrategy
{
	public:
		MIPSolutionLimitStrategyUnlimited(IMIPSolver *MIPSolver);
		~MIPSolutionLimitStrategyUnlimited();

		virtual bool updateLimit();
		virtual int getNewLimit();
		virtual int getInitialLimit();
};
