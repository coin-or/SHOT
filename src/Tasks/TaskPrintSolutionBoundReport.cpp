/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskPrintSolutionBoundReport.h"

TaskPrintSolutionBoundReport::TaskPrintSolutionBoundReport()
{
	itersSinceLastPrintout = 0;
	timeLastPrintout = 0;
}

TaskPrintSolutionBoundReport::~TaskPrintSolutionBoundReport()
{
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

		Output::getInstance().outputSummary(
			"                                                                                     ");

#ifdef _WIN32
		Output::getInstance().outputSummary(
			"ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ");
#else
		Output::getInstance().outputSummary(
			"─────────────────────────────────────────────────────────────────────────────────────");
#endif

		auto tmpLine = boost::format(" At %1% s the obj. bound is %|24t|[%2%, %3%] %|46t|with abs/rel gap %4% / %5%") % ProcessInfo::getInstance().getElapsedTime("Total") % UtilityFunctions::toStringFormat(objLB, "%.3f", true) % UtilityFunctions::toStringFormat(objUB, "%.3f", true) % UtilityFunctions::toStringFormat(absGap, "%.4f", true) % UtilityFunctions::toStringFormat(relGap, "%.4f", true);
		Output::getInstance().outputSummary(tmpLine.str());

		if (ProcessInfo::getInstance().solutionStatistics.numberOfConstraintsRemovedInPresolve> 0 || ProcessInfo::getInstance().solutionStatistics.numberOfVariableBoundsTightenedInPresolve > 0)
		{
			tmpLine = boost::format(" Presolve: %1% constraint(s) removed and %2% variable bounds tightened.") % ProcessInfo::getInstance().solutionStatistics.numberOfConstraintsRemovedInPresolve% ProcessInfo::getInstance().solutionStatistics.numberOfVariableBoundsTightenedInPresolve;
			Output::getInstance().outputSummary(tmpLine.str());
		}

		if (ProcessInfo::getInstance().interiorPts.size() > 1)
		{
			Output::getInstance().outputSummary(
				" Number of interior points: " + to_string(ProcessInfo::getInstance().interiorPts.size()));
		}

		if (ProcessInfo::getInstance().solutionStatistics.numberOfIntegerCuts > 0)
		{
			Output::getInstance().outputSummary(
				" Number of integer cuts added: " + to_string(ProcessInfo::getInstance().solutionStatistics.numberOfIntegerCuts));
		}

#ifdef _WIN32
		Output::getInstance().outputSummary("ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ");
#else
		Output::getInstance().outputSummary(
			"─────────────────────────────────────────────────────────────────────────────────────");
#endif

		Output::getInstance().outputSummary("");

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
