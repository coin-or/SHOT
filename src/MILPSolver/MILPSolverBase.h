#pragma once
#include <vector>
#include "OSInstance.h"
#include "IMILPSolver.h"
#include "SHOTSettings.h"
#include "../ProcessInfo.h"

class MILPSolverBase
{
	private:
		std::vector<GeneratedHyperplane> generatedHyperplanes;
		int addedHyperplanes = 0;
	protected:
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

		bool discreteVariablesActivated;
		bool cachedSolutionHasChanged;

		std::vector<SolutionPoint> lastSolutions;

		std::vector<double> lastLazyUpdateConstrSlacks;
		std::vector<double> lastSolutionConstrSlacks;

		//std::vector<E_HyperplaneSource> hyperplaneSource;
		//std::vector<bool> isConstraintLazy;

		virtual void startTimer();
		virtual void stopTimer();

	public:
		MILPSolverBase();
		~MILPSolverBase();

		virtual void createHyperplane(Hyperplane hyperplane);
		virtual void createInteriorHyperplane(Hyperplane hyperplane);
		virtual bool getDiscreteVariableStatus();
		virtual void populateSolutionPool() = 0;
		virtual std::vector<SolutionPoint> getAllVariableSolutions();
		virtual int getNumberOfSolutions() = 0;
		virtual std::vector<double> getVariableSolution(int i) = 0;
		virtual double getObjectiveValue(int i) = 0;
		virtual double getObjectiveValue();

		virtual std::vector<GeneratedHyperplane>* getGeneratedHyperplanes();
};
