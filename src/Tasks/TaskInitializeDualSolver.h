/*
 * TaskInitializeDualSolver.h
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#pragma once

#include "TaskBase.h"
#include "../ProcessInfo.h"
#include "../UtilityFunctions.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../MIPSolver/MIPSolverCplex.h"
#include "../MIPSolver/MIPSolverCplexLazy.h"
#include "../MIPSolver/MIPSolverCplexLazyOriginalCallback.h"
#include "../MIPSolver/MIPSolverGurobi.h"
#include "../MIPSolver/MIPSolverGurobiLazy.h"
#include "../MIPSolver/MIPSolverOsiCbc.h"

class TaskInitializeDualSolver: public TaskBase
{
	public:
		TaskInitializeDualSolver(ES_MIPSolver solver, bool useLazyStrategy);
		virtual ~TaskInitializeDualSolver();

		void run();
		virtual std::string getType();

	private:

};

