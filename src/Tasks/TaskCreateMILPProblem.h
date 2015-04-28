/*
 * TaskCreateMILPProblem.h
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#pragma once

#include <TaskBase.h>

#include "../ProcessInfo.h"
#include "../MILPSolver/IMILPSolver.h"
class TaskCreateMILPProblem: public TaskBase
{
	public:
		TaskCreateMILPProblem();
		virtual ~TaskCreateMILPProblem();

		virtual void run();

	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};

