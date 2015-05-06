/*
 * TaskSelectHyperplanePointsSolution.h
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

#include "../OptProblems/OptProblemOriginal.h"

class TaskSelectHyperplanePointsSolution: public TaskBase
{
	public:
		TaskSelectHyperplanePointsSolution();
		virtual ~TaskSelectHyperplanePointsSolution();

		virtual void run();
		virtual std::string getType();

	private:
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};

