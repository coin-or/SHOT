/*
 * TaskCreateMILPProblem.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include "TaskCreateMILPProblem.h"

TaskCreateMILPProblem::TaskCreateMILPProblem(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;

	ProcessInfo::getInstance().startTimer("Reformulation");

	ProcessInfo::getInstance().outputDebug("Creating MILP problem");

	MILPSolver->createLinearProblem(ProcessInfo::getInstance().originalProblem);

	MILPSolver->initializeSolverSettings();

	if (Settings::getInstance().getBoolSetting("Debug", "SHOTSolver"))
	{
		MILPSolver->writeProblemToFile(Settings::getInstance().getStringSetting("DebugPath", "SHOTSolver") + "/lp0.lp");
	}

	ProcessInfo::getInstance().outputDebug("MILP problem created");
	ProcessInfo::getInstance().stopTimer("Reformulation");

}

TaskCreateMILPProblem::~TaskCreateMILPProblem()
{
	// TODO Auto-generated destructor stub
}

void TaskCreateMILPProblem::run()
{
}

std::string TaskCreateMILPProblem::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
