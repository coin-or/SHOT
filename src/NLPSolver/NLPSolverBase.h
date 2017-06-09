#pragma once

#include "SHOTSettings.h"
#include "../ProcessInfo.h"
#include "INLPSolver.h"
#include "iterator"
#include "vector"

class NLPSolverBase: virtual public INLPSolver
{
	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

	protected:

		OSInstance* originalInstance;

		bool isProblemInitialized;

	public:

		virtual void setProblem(OSInstance * origInstance);
		virtual void initializeProblem();
		virtual E_NLPSolutionStatus solveProblem();

		virtual void saveProblemToFile(std::string fileName);

		virtual std::vector<double> getVariableLowerBounds();
		virtual std::vector<double> getVariableUpperBounds();
};
