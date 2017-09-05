/*
 * NLPSolverCuttingPlaneMinimax.h
 *
 *  Created on: Apr 16, 2015
 *      Author: alundell
 */

#pragma once

#include "NLPSolverBase.h"
#include "../OptProblems/OptProblemNLPSHOTMinimax.h"
#include "../OptProblems/OptProblemNLPMinimax.h"
#include "../MILPSolver/MILPSolverCplex.h"
#include "../MILPSolver/MILPSolverGurobi.h"
#include "../MILPSolver/MILPSolverOsiCbc.h"

#include "boost/math/tools/minima.hpp"

class NLPSolverCuttingPlaneMinimax: public NLPSolverBase
{
	public:
		NLPSolverCuttingPlaneMinimax();
		virtual ~NLPSolverCuttingPlaneMinimax();

		//virtual void saveProblemModelToFile(std::string fileName);

		virtual void setStartingPoint(std::vector<int> variableIndexes, std::vector<double> variableValues);
		virtual void clearStartingPoint();

		virtual bool isObjectiveFunctionNonlinear();
		virtual int getObjectiveFunctionVariableIndex();

		virtual std::vector<double> getCurrentVariableLowerBounds();
		virtual std::vector<double> getCurrentVariableUpperBounds();

	private:

		SHOTSettings::Settings *settings;
		//ProcessInfo *processInfo;
		//OSInstance* originalInstance;

		//OptProblemNLPMinimax *NLPProblem;
		IMILPSolver *LPSolver;

		virtual double getSolution(int i);
		virtual std::vector<double> getSolution();
		virtual double getObjectiveValue();

		virtual bool createProblemInstance(OSInstance * origInstance);

		virtual void fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues);

		virtual void unfixVariables();

		//virtual void initializeProblem(OSInstance * origInstance);

		virtual E_NLPSolutionStatus solveProblemInstance();

		//virtual E_NLPSolutionStatus solveProblem();
		//virtual E_NLPSolutionStatus fixVariablesAndSolve(std::vector<int> variableIndexes,
		//std::vector<double> variableValues);

		//virtual void saveProblemToFile(std::string fileName);
		virtual void saveOptionsToFile(std::string fileName);

		bool isProblemCreated;

		std::vector<double> solution;
		double objectiveValue;
};

