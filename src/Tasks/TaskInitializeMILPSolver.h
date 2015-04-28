/*
 * TaskInitializeMILPSolver.h
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#pragma once

#include <TaskBase.h>
#include "../ProcessInfo.h"
#include "../MILPSolver/IMILPSolver.h"
#include "../MILPSolver/MILPSolverCplex.h"
#include "../MILPSolver/MILPSolverGurobi.h"
#include "../MILPSolver/MILPSolverOsiCbc.h"

class TaskInitializeMILPSolver: public TaskBase
{
	public:
		TaskInitializeMILPSolver();
		virtual ~TaskInitializeMILPSolver();

		void run();

	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};

