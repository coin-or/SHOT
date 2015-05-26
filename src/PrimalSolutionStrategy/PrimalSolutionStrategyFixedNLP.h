/*
 * PrimalSolutionStrategyFixedNLP.h
 *
 *  Created on: Mar 6, 2015
 *      Author: alundell
 */

#pragma once
#include <vector>
#include <PrimalSolutionStrategyBase.h>
#include "NLPSolverIPOptBase.h"
#include "../OptProblems/OptProblemNLPRelaxed.h"

class PrimalSolutionStrategyFixedNLP: public PrimalSolutionStrategyBase, public INLPSolver, public NLPSolverIPOptBase
{
	public:
		PrimalSolutionStrategyFixedNLP();
		virtual ~PrimalSolutionStrategyFixedNLP();

		virtual bool runStrategy();

		virtual bool createProblem(OSInstance * origInstance);
		virtual bool solveProblem();
		virtual void saveProblemModelToFile(std::string fileName);

	protected:
		using PrimalSolutionStrategyBase::processInfo;
		using PrimalSolutionStrategyBase::settings;

	private:
		OptProblemNLPRelaxed *NLPProblem;
		bool isOriginalRelaxedPointTested;

		std::vector<int> discreteVariableIndexes;
		std::vector<std::vector<double>> testedPoints;

		double originalNLPTime;
		double originalNLPIter;
};
