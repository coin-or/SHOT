/*
 * NLPSolverCuttingPlane.h
 *
 *  Created on: Apr 16, 2015
 *      Author: alundell
 */

#pragma once

#include <INLPSolver.h>
#include <NLPSolverBase.h>
#include "../OptProblems/OptProblemNLPSHOTMinimax.h"
#include "../OptProblems/OptProblemNLPMinimax.h"
#include "../MILPSolver/MILPSolverCplex.h"
#include "../MILPSolver/MILPSolverGurobi.h"
#include "../MILPSolver/MILPSolverOsiCbc.h"

#include <boost/math/tools/minima.hpp>

class NLPSolverCuttingPlane: public INLPSolver, NLPSolverBase
{
	public:
		NLPSolverCuttingPlane();
		virtual ~NLPSolverCuttingPlane();

		virtual bool createProblem(OSInstance * origInstance);
		virtual bool solveProblem();
		virtual void saveProblemModelToFile(std::string fileName);

	private:

		OptProblemNLPMinimax *NLPProblem;
		IMILPSolver *MILPSolver;
};
