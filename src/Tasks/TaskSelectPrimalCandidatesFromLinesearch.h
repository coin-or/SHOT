/*
 * TaskSelectPrimalCandidatesFromLinesearch.h
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#pragma once

#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MILPSolver/IMILPSolver.h"

//

class TaskSelectPrimalCandidatesFromLinesearch: public TaskBase
{
	public:
		TaskSelectPrimalCandidatesFromLinesearch();
		virtual ~TaskSelectPrimalCandidatesFromLinesearch();
		virtual void run();
		virtual void run(vector<SolutionPoint> solPoints);

		virtual std::string getType();
	private:

};

