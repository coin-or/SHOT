#pragma once
#include "NLPSolverIPOptBase.h"
#include "../OptProblems/OptProblemNLPMinimax.h"
#include "OSnl2OS.h"
#include "../UtilityFunctions.h"

#include "IpIpoptCalculatedQuantities.hpp"
#include "IpIpoptData.hpp"
#include "IpTNLPAdapter.hpp"
#include "IpOrigIpoptNLP.hpp"

class NLPSolverIPOptMinimax: public INLPSolver, public NLPSolverIPOptBase
{
	public:
		NLPSolverIPOptMinimax();
		~NLPSolverIPOptMinimax();

		virtual bool createProblem(OSInstance * origInstance);
		virtual bool solveProblem();
		virtual void saveProblemModelToFile(std::string fileName);

	private:
		//OSOption* osOption;

		OptProblemNLPMinimax *NLPProblem;
		//DefaultSolver *NLPSolver;
		//std::vector<double> solution;
		//SHOTSettings::Settings *settings;
		//ProcessInfo *processInfo;
		//bool isPointValueCached;

};
