#pragma once

#include "SHOTSettings.h"
#include "../ProcessInfo.h"
#include "IMILPSolver.h"

class RelaxationStrategyBase
{
	public:
		//RelaxationStrategyBase();
		//~RelaxationStrategyBase();

	protected:

		bool isRelaxedSolutionEpsilonValid();
		bool isRelaxedSolutionInterior();
		bool isCurrentToleranceReached();
		bool isGapReached();

		//IMILPSolver *MILPSolver;
		//bool relaxationActive;

};

