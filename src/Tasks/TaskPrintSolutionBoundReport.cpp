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

	if (itersSinceLastPrintout > 20 || currElapsedTime - timeLastPrintout > 10.0)
	{
		double absGap = processInfo->getAbsoluteObjectiveGap();
		double relGap = processInfo->getRelativeObjectiveGap();
		auto objBounds = processInfo->getCorrectedObjectiveBounds();
		double objLB = objBounds.first;
		double objUB = objBounds.second;

		auto tmpLine = boost::format(" At %1% s the obj. bound is %|24t|[%2%, %3%] %|46t|with abs/rel gap %4% / %5%")
				% processInfo->getElapsedTime("Total") % objLB % objUB % absGap % relGap;

		processInfo->outputSummary("");

		processInfo->outputSummary(
				"─────────────────────────────────────────────────────────────────────────────────────");

		processInfo->outputSummary(tmpLine.str());
		processInfo->outputSummary(
				"─────────────────────────────────────────────────────────────────────────────────────");

		if (processInfo->interiorPts.size() > 1)
		{
			processInfo->outputSummary("Number of interior points: " + to_string(processInfo->interiorPts.size()));
		}

		processInfo->outputSummary("");

		itersSinceLastPrintout = 0;
		timeLastPrintout = currElapsedTime;
	}
	else
	{
		itersSinceLastPrintout++;
	}
}

std::string TaskPrintSolutionBoundReport::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
