/*
 * TaskSelectHyperplanePointsIndividualLinesearch.h
 *
 *  Created on: Feb 3, 2017
 *      Author: alundell
 */

#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"
#include "../OptProblems/OptProblemOriginal.h"

class TaskSelectHyperplanePointsIndividualLinesearch: public TaskBase
{
	public:
		TaskSelectHyperplanePointsIndividualLinesearch();
		virtual ~TaskSelectHyperplanePointsIndividualLinesearch();

		virtual void run();
		virtual void run(vector<SolutionPoint> solPoints);

		virtual std::string getType();
	private:

		std::vector<int> nonlinearConstraintIdxs;
};

