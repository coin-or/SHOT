#pragma once
#include "IMILPSolutionLimitStrategy.h"

class MILPSolutionLimitStrategyUnlimited: public IMILPSolutionLimitStrategy
{
	public:
		MILPSolutionLimitStrategyUnlimited(IMILPSolver *MILPSolver);
		~MILPSolutionLimitStrategyUnlimited();

		virtual bool updateLimit();
		virtual int getNewLimit();
		virtual int getInitialLimit();
};
