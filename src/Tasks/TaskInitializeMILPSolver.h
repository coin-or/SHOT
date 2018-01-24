/*
 * TaskInitializeMILPSolver.h
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#pragma once

#include <MILPSolverCplexLazy.h>
#include "TaskBase.h"
#include "../ProcessInfo.h"
#include "../UtilityFunctions.h"
#include "../MILPSolver/IMILPSolver.h"
#include "../MILPSolver/MILPSolverCplex.h"
#include "../MILPSolver/MILPSolverCplexLazy.h"
#include "../MILPSolver/MILPSolverCplexLazyOriginalCB.h"
#include "../MILPSolver/MILPSolverGurobi.h"
#include "../MILPSolver/MILPSolverGurobiLazy.h"
#include "../MILPSolver/MILPSolverOsiCbc.h"

class TaskInitializeMILPSolver: public TaskBase
{
	public:
		TaskInitializeMILPSolver(ES_MILPSolver solver, bool useLazyStrategy);
		virtual ~TaskInitializeMILPSolver();

		void run();
		virtual std::string getType();

	private:

};

