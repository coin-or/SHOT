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
		TaskCreateMILPProblem(IMILPSolver *MILPSolver);
		virtual ~TaskCreateMILPProblem();

		virtual void run();
		virtual std::string getType();

	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

		IMILPSolver *MILPSolver;
};

