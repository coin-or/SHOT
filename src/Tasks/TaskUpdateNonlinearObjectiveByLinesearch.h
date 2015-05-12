/*
 * TaskSelectPrimalCandidatesFromLinesearch.h
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#pragma once

#include <TaskBase.h>
#include "../ProcessInfo.h"

#include "../LinesearchMethod/ILinesearchMethod.h"
#include "../LinesearchMethod/LinesearchMethodBisection.h"
#include "../LinesearchMethod/LinesearchMethodBoost.h"
#include "../MILPSolver/IMILPSolver.h"

#include <boost/math/tools/roots.hpp>

class TaskUpdateNonlinearObjectiveByLinesearch: public TaskBase
{
	public:
		TaskUpdateNonlinearObjectiveByLinesearch();
		virtual ~TaskUpdateNonlinearObjectiveByLinesearch();
		virtual void run();
		virtual std::string getType();
	private:
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
		ILinesearchMethod *linesearchMethod;
};

