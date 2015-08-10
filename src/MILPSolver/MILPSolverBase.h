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
		bool cachedSolutionHasChanged;

		std::vector<SolutionPoint> lastSolutions;

		std::vector<double> lastLazyUpdateConstrSlacks;
		std::vector<double> lastSolutionConstrSlacks;

		virtual void startTimer();
		virtual void stopTimer();

	public:
		MILPSolverBase();
		~MILPSolverBase();

		virtual void createHyperplane(int constrIdx, std::vector<double> point);
		virtual bool getDiscreteVariableStatus();
		virtual void populateSolutionPool() = 0;
		virtual std::vector<SolutionPoint> getAllVariableSolutions();
		virtual int getNumberOfSolutions() = 0;
		virtual std::vector<double> getVariableSolution(int i) = 0;
		virtual double getObjectiveValue(int i) = 0;
		virtual double getObjectiveValue();
};
