/*
 * NLPSolverCuttingPlaneMinimax.h
 *
 *  Created on: Apr 16, 2015
 *      Author: alundell
 */

#pragma once

#include "NLPSolverBase.h"
#include "../OptProblems/OptProblemNLPMinimax.h"
#include "../MIPSolver/MIPSolverCplex.h"
#include "../MIPSolver/MIPSolverGurobi.h"
#include "../MIPSolver/MIPSolverOsiCbc.h"

#include "boost/math/tools/minima.hpp"

class NLPSolverCuttingPlaneMinimax: public NLPSolverBase
{
	public:
		NLPSolverCuttingPlaneMinimax();
		virtual ~NLPSolverCuttingPlaneMinimax();

		virtual void setStartingPoint(std::vector<int> variableIndexes, std::vector<double> variableValues);
		virtual void clearStartingPoint();

		virtual bool isObjectiveFunctionNonlinear();
		virtual int getObjectiveFunctionVariableIndex();

		virtual std::vector<double> getCurrentVariableLowerBounds();
		virtual std::vector<double> getCurrentVariableUpperBounds();

	private:

		IMIPSolver *LPSolver;

		virtual double getSolution(int i);
		virtual std::vector<double> getSolution();
		virtual double getObjectiveValue();

		virtual bool createProblemInstance(OSInstance * origInstance);

		virtual void fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues);

		virtual void unfixVariables();

		virtual E_NLPSolutionStatus solveProblemInstance();

		virtual void saveOptionsToFile(std::string fileName);

		bool isProblemCreated;

		std::vector<double> solution;
		double objectiveValue;
};

