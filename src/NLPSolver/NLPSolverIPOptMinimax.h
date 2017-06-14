#pragma once
#include "NLPSolverIPOptBase.h"
#include "../OptProblems/OptProblemNLPMinimax.h"

class NLPSolverIPOptMinimax: public NLPSolverBase, public NLPSolverIPOptBase
{
	public:
		NLPSolverIPOptMinimax();
		~NLPSolverIPOptMinimax();

		std::vector<double> getSolution();

	protected:
		bool createProblemInstance(OSInstance * origInstance);

		void setSolverSpecificInitialSettings();

	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
