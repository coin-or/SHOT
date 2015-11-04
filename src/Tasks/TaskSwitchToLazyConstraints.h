/*
 * TaskSwitchToLazyConstraints.h
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#pragma once

#include <TaskBase.h>
#include "../ProcessInfo.h"
#include "../MILPSolver/IMILPSolver.h"
#include "../MILPSolver/MILPSolverBase.h"

class TaskSwitchToLazyConstraints: public TaskBase
{
	public:
		TaskSwitchToLazyConstraints();
		virtual ~TaskSwitchToLazyConstraints();

		void run();
		virtual std::string getType();

	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};

