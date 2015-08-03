/*
 * TaskAddHyperplanes.h
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#pragma once

#include <TaskBase.h>

#include "../OptProblems/OptProblemOriginal.h"
#include "../MILPSolver/IMILPSolver.h"

class TaskAddHyperplanes: public TaskBase
{
	public:
		TaskAddHyperplanes();
		virtual ~TaskAddHyperplanes();

		virtual void run();

		virtual std::string getType();
	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

		int itersWithoutAddedHPs;
};

