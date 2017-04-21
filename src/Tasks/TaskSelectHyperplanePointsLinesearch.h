/*
 * TaskSelectHyperplanePoints.h
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

#include "../LinesearchMethod/ILinesearchMethod.h"
#include "../LinesearchMethod/LinesearchMethodBisection.h"
#include "../LinesearchMethod/LinesearchMethodBoost.h"

class TaskSelectHyperplanePointsLinesearch: public TaskBase
{
	public:
		TaskSelectHyperplanePointsLinesearch();
		virtual ~TaskSelectHyperplanePointsLinesearch();

		virtual void run();
		virtual void run(vector<SolutionPoint> solPoints);

		virtual std::string getType();
	private:
		ILinesearchMethod *linesearchMethod;

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};

