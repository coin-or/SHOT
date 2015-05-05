/*
 * TaskPrintSolutionBoundReport.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include <TaskPrintSolutionBoundReport.h>

TaskPrintSolutionBoundReport::TaskPrintSolutionBoundReport()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskPrintSolutionBoundReport::~TaskPrintSolutionBoundReport()
{
	// TODO Auto-generated destructor stub
}

void TaskPrintSolutionBoundReport::run()
{
	double currElapsedTime = processInfo->getElapsedTime("Total");

	if (itersSinceLastPrintout > 4 || currElapsedTime - timeLastPrintout > 10.0)
	{
		double absGap = processInfo->getAbsoluteObjectiveGap();
		double relGap = processInfo->getRelativeObjectiveGap();
		auto objBounds = processInfo->getCorrectedObjectiveBounds();
		double objLB = objBounds.first;
		double objUB = objBounds.second;

		auto tmpLine = boost::format("    Current obj. bound: %|24t|[%1%, %2%] %|46t|Gap abs/rel: %3% / %4%") % objLB
				% objUB % absGap % relGap;

		processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;

		itersSinceLastPrintout = 0;
		timeLastPrintout = currElapsedTime;
	}
	else
	{
		itersSinceLastPrintout++;
	}
}
