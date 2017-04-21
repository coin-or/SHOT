/*
 * TaskSelectHyperplanePointsIndividualLinesearch.h
 *
 *  Created on: Feb 3, 2017
 *      Author: alundell
 */

#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

#include "../LinesearchMethod/ILinesearchMethod.h"
#include "../LinesearchMethod/LinesearchMethodBisection.h"
#include "../LinesearchMethod/LinesearchMethodBoost.h"

class TaskSelectHyperplanePointsIndividualLinesearch: public TaskBase
{
	public:
		TaskSelectHyperplanePointsIndividualLinesearch();
		virtual ~TaskSelectHyperplanePointsIndividualLinesearch();

		virtual void run();
		virtual void run(vector<SolutionPoint> solPoints);

		virtual std::string getType();
	private:
		ILinesearchMethod *linesearchMethod;

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

		std::vector<int> nonlinearConstraintIdxs;
};

