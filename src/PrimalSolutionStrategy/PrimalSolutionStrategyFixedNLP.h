/*
 * PrimalSolutionStrategyFixedNLP.h
 *
 *  Created on: Mar 6, 2015
 *      Author: alundell
 */

#pragma once
#include "vector"
#include "PrimalSolutionStrategyBase.h"
#include "../NLPSolver/INLPSolver.h"
#include "../NLPSolver/NLPSolverIpoptRelaxed.h"
#include "../NLPSolver/NLPSolverCuttingPlaneRelaxed.h"

#ifdef HAS_GAMS
#include "../NLPSolver/NLPSolverGAMS.h"
#endif

#include "../Tasks/TaskSelectHyperplanePointsLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsIndividualLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsSolution.h"

class PrimalSolutionStrategyFixedNLP: public PrimalSolutionStrategyBase
{
	public:
		PrimalSolutionStrategyFixedNLP();
		virtual ~PrimalSolutionStrategyFixedNLP();

		virtual bool runStrategy();

	protected:

	private:
		INLPSolver *NLPSolver;

		std::vector<int> discreteVariableIndexes;
		std::vector<std::vector<double>> testedPoints;
		std::vector<double> fixPoint;

		double originalNLPTime;
		double originalNLPIter;

		std::vector<double> originalLBs;
		std::vector<double> originalUBs;

		TaskBase *taskSelectHPPts;
};

