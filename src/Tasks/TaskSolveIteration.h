/*
 * TaskSolveIteration.h
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

#include "../MILPSolver/IMILPSolver.h"

class TaskSolveIteration: public TaskBase
{
	public:
		TaskSolveIteration();
		virtual ~TaskSolveIteration();

		virtual void run();
		virtual std::string getType();
	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};

