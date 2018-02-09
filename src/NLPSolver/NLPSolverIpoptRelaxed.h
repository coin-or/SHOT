#pragma once
#include "NLPSolverIpoptBase.h"
#include "../OptProblems/OptProblemNLPRelaxed.h"
//#include "../UtilityFunctions.h"

class NLPSolverIpoptRelaxed: public NLPSolverBase, public NLPSolverIpoptBase
{
	public:
		NLPSolverIpoptRelaxed();
		~NLPSolverIpoptRelaxed();

		virtual std::vector<double> getSolution();

	protected:

		bool createProblemInstance(OSInstance * origInstance);

		virtual void setSolverSpecificInitialSettings();

	private:

};
