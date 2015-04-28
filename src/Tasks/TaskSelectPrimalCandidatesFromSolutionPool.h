/*
 * TaskSelectPrimalCandidatesFromSolutionPool.h
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#pragma once

#include <TaskBase.h>
#include "../ProcessInfo.h"
#include "../MILPSolver/IMILPSolver.h"

class TaskSelectPrimalCandidatesFromSolutionPool: public TaskBase
{
	public:
		TaskSelectPrimalCandidatesFromSolutionPool();
		virtual ~TaskSelectPrimalCandidatesFromSolutionPool();

		virtual void run();
	private:
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
