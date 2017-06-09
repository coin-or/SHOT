/*
 * TaskCreateMILPProblem.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include <TaskCreateMILPProblem.h>

TaskCreateMILPProblem::TaskCreateMILPProblem(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("Reformulation");

	processInfo->outputDebug("Creating MILP problem");

	MILPSolver->createLinearProblem(processInfo->originalProblem);

	MILPSolver->initializeSolverSettings();

	if (settings->getBoolSetting("Debug", "SHOTSolver"))
	{
		MILPSolver->writeProblemToFile(settings->getStringSetting("DebugPath", "SHOTSolver") + "/lp0.lp");
	}

	processInfo->outputDebug("MILP problem created");
	processInfo->stopTimer("Reformulation");

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
