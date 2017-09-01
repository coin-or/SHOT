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
#include "../NLPSolver/NLPSolverIPOptRelaxed.h"
#include "../NLPSolver/NLPSolverCuttingPlaneRelaxed.h"
#include "../NLPSolver/NLPSolverGAMS.h"

class PrimalSolutionStrategyFixedNLP: public PrimalSolutionStrategyBase
{
	public:
		PrimalSolutionStrategyFixedNLP();
		virtual ~PrimalSolutionStrategyFixedNLP();

		virtual bool runStrategy();

	protected:
		using PrimalSolutionStrategyBase::processInfo;
		using PrimalSolutionStrategyBase::settings;

	private:
		INLPSolver *NLPSolver;

		std::vector<int> discreteVariableIndexes;
		std::vector<std::vector<double>> testedPoints;
		std::vector<double> fixPoint;

		double originalNLPTime;
		double originalNLPIter;

		std::vector<double> originalLBs;
		std::vector<double> originalUBs;

};

