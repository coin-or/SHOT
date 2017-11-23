/*
 * TaskPrintSolutionBoundReport.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include "TaskPrintSolutionBoundReport.h"

TaskPrintSolutionBoundReport::TaskPrintSolutionBoundReport()
{

	itersSinceLastPrintout = 0;
	timeLastPrintout = 0;

}

TaskPrintSolutionBoundReport::~TaskPrintSolutionBoundReport()
{
	// TODO Auto-generated destructor stub
}

void TaskPrintSolutionBoundReport::run()
{
	double currElapsedTime = ProcessInfo::getInstance().getElapsedTime("Total");

	if (itersSinceLastPrintout > 20 || currElapsedTime - timeLastPrintout > 10.0)
	{
		double absGap = ProcessInfo::getInstance().getAbsoluteObjectiveGap();
		double relGap = ProcessInfo::getInstance().getRelativeObjectiveGap();
		auto objBounds = ProcessInfo::getInstance().getCorrectedObjectiveBounds();
		double objLB = objBounds.first;
		double objUB = objBounds.second;

		ProcessInfo::getInstance().outputSummary(
				"                                                                                     ");

#ifdef _WIN32
		ProcessInfo::getInstance().outputSummary(
				"ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ");
#else
		ProcessInfo::getInstance().outputSummary(
				"─────────────────────────────────────────────────────────────────────────────────────");
#endif

		auto tmpLine = boost::format(" At %1% s the obj. bound is %|24t|[%2%, %3%] %|46t|with abs/rel gap %4% / %5%")
				% ProcessInfo::getInstance().getElapsedTime("Total") % objLB % objUB % absGap % relGap;
		ProcessInfo::getInstance().outputSummary(tmpLine.str());

		if (ProcessInfo::getInstance().numConstraintsRemovedInPresolve > 0
				|| ProcessInfo::getInstance().numVariableBoundsTightenedInPresolve > 0)
		{
			tmpLine = boost::format(" Presolve: %1% constraint(s) removed and %2% variable bounds tightened.")
					% ProcessInfo::getInstance().numConstraintsRemovedInPresolve
					% ProcessInfo::getInstance().numVariableBoundsTightenedInPresolve;
			ProcessInfo::getInstance().outputSummary(tmpLine.str());
		}

		if (ProcessInfo::getInstance().interiorPts.size() > 1)
		{
			ProcessInfo::getInstance().outputSummary(
					" Number of interior points: " + to_string(ProcessInfo::getInstance().interiorPts.size()));
		}

		if (ProcessInfo::getInstance().numIntegerCutsAdded > 0)
		{
			ProcessInfo::getInstance().outputSummary(
					" Number of integer cuts added: " + to_string(ProcessInfo::getInstance().numIntegerCutsAdded));
		}

#ifdef _WIN32
		ProcessInfo::getInstance().outputSummary("ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ");
#else
		ProcessInfo::getInstance().outputSummary(
				"─────────────────────────────────────────────────────────────────────────────────────");
#endif

		ProcessInfo::getInstance().outputSummary("");

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
