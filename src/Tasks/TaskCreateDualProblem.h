/*
 * TaskCreateDualProblem.h
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#pragma once

#include "TaskBase.h"

#include "../ProcessInfo.h"
#include "../MIPSolver/IMIPSolver.h"
class TaskCreateDualProblem: public TaskBase
{
	public:
		TaskCreateDualProblem(IMIPSolver *MIPSolver);
		virtual ~TaskCreateDualProblem();

		virtual void run();
		virtual std::string getType();

	private:

		IMIPSolver *MIPSolver;
};

