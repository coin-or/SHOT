#pragma once
#include <SHOTSettings.h>
#include "../ProcessInfo.h"
#include "IMILPSolver.h"

class IMILPSolutionLimitStrategy
{
	public:
		//IMILPSolutionLimitStrategy();
		virtual ~IMILPSolutionLimitStrategy()
		{
		}
		;

		virtual bool updateLimit() = 0;

		virtual int getNewLimit() = 0;

		virtual int getInitialLimit() = 0;

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
		IMILPSolver *MILPSolver;

	protected:
};
