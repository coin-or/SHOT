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
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
		bool isRelaxedSolutionEpsilonValid();
		bool isRelaxedSolutionInterior();
		bool isCurrentToleranceReached();

		//IMILPSolver *MILPSolver;
		//bool relaxationActive;

};

