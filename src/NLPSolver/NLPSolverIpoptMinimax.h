#pragma once
#include "NLPSolverIpoptBase.h"
#include "../OptProblems/OptProblemNLPMinimax.h"

class NLPSolverIpoptMinimax: public NLPSolverBase, public NLPSolverIpoptBase
{
	public:
		NLPSolverIpoptMinimax();
		~NLPSolverIpoptMinimax();

		std::vector<double> getSolution();

	protected:
		bool createProblemInstance(OSInstance * origInstance);

		void setSolverSpecificInitialSettings();

	private:

};
