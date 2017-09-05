#pragma once
#include "NLPSolverIPOptBase.h"
#include "../OptProblems/OptProblemNLPRelaxed.h"
//#include "../UtilityFunctions.h"

class NLPSolverIPOptRelaxed: public NLPSolverBase, public NLPSolverIPOptBase
{
	public:
		NLPSolverIPOptRelaxed();
		~NLPSolverIPOptRelaxed();

		virtual std::vector<double> getSolution();

	protected:

		bool createProblemInstance(OSInstance * origInstance);

		virtual void setSolverSpecificInitialSettings();

	private:

		SHOTSettings::Settings *settings;
		//ProcessInfo *processInfo;
};
