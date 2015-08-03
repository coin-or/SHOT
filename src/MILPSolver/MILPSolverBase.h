#pragma once
#include <vector>
#include "OSInstance.h"
#include "IMILPSolver.h"
#include "SHOTSettings.h"
#include "../ProcessInfo.h"

class MILPSolverBase
{
	protected:
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

		bool discreteVariablesActivated;

		std::vector<double> lastLazyUpdateConstrSlacks;
		std::vector<double> lastSolutionConstrSlacks;

		virtual void startTimer();
		virtual void stopTimer();

	public:
		MILPSolverBase();
		~MILPSolverBase();

		virtual bool getDiscreteVariableStatus();
		virtual void createHyperplane(int constrIdx, std::vector<double> point);
};
