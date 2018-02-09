#pragma once

#include "SHOTSettings.h"
#include "../ProcessInfo.h"
#include "IMIPSolver.h"

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

		//IMIPSolver *MIPSolver;
		//bool relaxationActive;

};

