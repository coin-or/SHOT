/*
 * TaskSolveIteration.h
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MIPSolver/IMIPSolver.h"

class TaskSolveIteration: public TaskBase
{
	public:
		TaskSolveIteration(IMIPSolver *MIPSolver);
		virtual ~TaskSolveIteration();

		virtual void run();
		virtual std::string getType();
	private:

		IMIPSolver *MIPSolver;
};

