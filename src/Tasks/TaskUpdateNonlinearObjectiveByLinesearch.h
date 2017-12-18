/*
 * TaskSelectPrimalCandidatesFromLinesearch.h
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#pragma once

#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../OptProblems/OptProblemOriginal.h"

class TaskUpdateNonlinearObjectiveByLinesearch: public TaskBase
{
	public:
		TaskUpdateNonlinearObjectiveByLinesearch();
		virtual ~TaskUpdateNonlinearObjectiveByLinesearch();
		virtual void run();
		virtual std::string getType();

		bool updateObjectiveInPoint(SolutionPoint &solution);

	private:

};

