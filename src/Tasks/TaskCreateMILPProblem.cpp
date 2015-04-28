/*
 * TaskCreateMILPProblem.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include <TaskCreateMILPProblem.h>

TaskCreateMILPProblem::TaskCreateMILPProblem()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("Reformulation");

	processInfo->logger.message(2) << "Creating MILP problem" << CoinMessageEol;

	processInfo->MILPSolver->createLinearProblem(processInfo->originalProblem);

	if (settings->getBoolSetting("Debug", "SHOTSolver")) processInfo->MILPSolver->writeProblemToFile(
			settings->getStringSetting("DebugPath", "SHOTSolver") + "/lp0.lp");

	processInfo->logger.message(2) << "MILP problem created" << CoinMessageEol;
	processInfo->stopTimer("Reformulation");

}

TaskCreateMILPProblem::~TaskCreateMILPProblem()
{
	// TODO Auto-generated destructor stub
}

void TaskCreateMILPProblem::run()
{
}
