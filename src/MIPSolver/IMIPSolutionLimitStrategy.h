#pragma once
#include <SHOTSettings.h>
#include "../ProcessInfo.h"
#include "IMIPSolver.h"

class IMIPSolutionLimitStrategy
{
	public:
		//IMIPSolutionLimitStrategy();
		virtual ~IMIPSolutionLimitStrategy()
		{
		}
		;

		virtual bool updateLimit() = 0;

		virtual int getNewLimit() = 0;

		virtual int getInitialLimit() = 0;

		IMIPSolver *MIPSolver;

	protected:
};
