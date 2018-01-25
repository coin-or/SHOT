#include "MILPSolverCallbackBase.h"


bool MILPSolverCallbackBase::checkFixedNLPStrategy(SolutionPoint point)
{
	if (!Settings::getInstance().getBoolSetting("PrimalStrategyFixedNLP", "PrimalBound"))
	{
		return (false);
	}

	ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
	ProcessInfo::getInstance().startTimer("PrimalBoundSearchNLP");

	bool callNLPSolver = false;

	auto userSettingStrategy = Settings::getInstance().getIntSetting("NLPFixedStrategy", "PrimalBound");
	auto userSetting = Settings::getInstance().getIntSetting("NLPFixedSource", "PrimalBound");

	if (abs(point.objectiveValue - ProcessInfo::getInstance().getDualBound()) < 0.1)
	{
		callNLPSolver = true;
	}
	else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::AlwaysUse))
	{
		callNLPSolver = true;
	}
	else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTime)
			|| userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
	{
		if (ProcessInfo::getInstance().itersMILPWithoutNLPCall
				>= Settings::getInstance().getIntSetting("NLPFixedMaxIters", "PrimalBound"))
		{
			ProcessInfo::getInstance().outputInfo(
					"     Activating fixed NLP primal strategy since max iterations since last call has been reached.");
			callNLPSolver = true;
		}
		else if (ProcessInfo::getInstance().getElapsedTime("Total") - ProcessInfo::getInstance().solTimeLastNLPCall
				> Settings::getInstance().getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound"))
		{
			ProcessInfo::getInstance().outputInfo(
					"     Activating fixed NLP primal strategy since max time limit since last call has been reached.");
			callNLPSolver = true;
		}
	}

	if (!callNLPSolver)
	{
		ProcessInfo::getInstance().itersMILPWithoutNLPCall++;
	}

	ProcessInfo::getInstance().stopTimer("PrimalBoundSearchNLP");
	ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");

	return (callNLPSolver);
}

void MILPSolverCallbackBase::printIterationReport(SolutionPoint solution, std::string threadId, std::string bestBound, std::string openNodes)
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->iterationNumber - lastHeaderIter > 100)
	{
		lastHeaderIter = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
		ProcessInfo::getInstance().tasks->getTask("PrintIterHeader")->run();
	}

	std::stringstream tmpType;
	tmpType << "LazyCB (" << threadId << ")";

	std::string hyperplanesExpr;

	if (this->lastNumAddedHyperplanes > 0)
	{
		hyperplanesExpr = "+" + to_string(this->lastNumAddedHyperplanes) + " = " + to_string(currIter->totNumHyperplanes);
	}
	else
	{
		hyperplanesExpr = " ";
	}

	auto tmpConstrExpr = UtilityFunctions::toStringFormat(solution.maxDeviation.value, "%.5f");

	if (solution.maxDeviation.idx != -1)
	{
		tmpConstrExpr = ProcessInfo::getInstance().originalProblem->getConstraintNames()[solution.maxDeviation.idx] + ": " + tmpConstrExpr;
	}
	else
	{
		tmpConstrExpr = ProcessInfo::getInstance().originalProblem->getConstraintNames().back() + ": " + tmpConstrExpr;
	}

	std::string primalBoundExpr;
	std::string dualBoundExpr;
	std::string objExpr;

	primalBoundExpr = UtilityFunctions::toStringFormat(
		ProcessInfo::getInstance().getPrimalBound(), "%.3f", true);

	objExpr = UtilityFunctions::toStringFormat(solution.objectiveValue, "%.3f", true);
	dualBoundExpr = bestBound;

	auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % to_string(currIter->iterationNumber) % tmpType.str() % hyperplanesExpr % dualBoundExpr % objExpr % primalBoundExpr % tmpConstrExpr;

	ProcessInfo::getInstance().outputSummary(tmpLine.str());

	double timeStamp = ProcessInfo::getInstance().getElapsedTime("Total");

	if (ProcessInfo::getInstance().getCurrentIteration()->iterationNumber - lastSummaryIter > 50 || timeStamp - lastSummaryTimeStamp > 5)
	{
		lastSummaryIter = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
		lastSummaryTimeStamp = timeStamp;
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

		auto tmpLineSummary = boost::format(" At %1% s the obj. bound is %|24t|[%2%, %3%] %|46t|with abs/"
											"rel gap %4% / %5%") %
							  ProcessInfo::getInstance().getElapsedTime("Total") % UtilityFunctions::toStringFormat(objLB, "%.3f", true) % UtilityFunctions::toStringFormat(objUB, "%.3f", true) % UtilityFunctions::toStringFormat(absGap, "%.4f", true) % UtilityFunctions::toStringFormat(relGap, "%.4f", true);

		ProcessInfo::getInstance().outputSummary(tmpLineSummary.str());

		std::stringstream tmpLine;

		tmpLine << " Number of open nodes: " << openNodes << ".";

		if (ProcessInfo::getInstance().interiorPts.size() > 1)
		{
			tmpLine << " Number of interior points: " << ProcessInfo::getInstance().interiorPts.size() << ".";
		}

		if (ProcessInfo::getInstance().numIntegerCutsAdded > 1)
		{
			tmpLine << " Number of integer cuts: " << ProcessInfo::getInstance().numIntegerCutsAdded << ".";
		}

		ProcessInfo::getInstance().outputSummary(tmpLine.str());

#ifdef _WIN32
		ProcessInfo::getInstance().outputSummary(
			"ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ");
#else
		ProcessInfo::getInstance().outputSummary(
			"─────────────────────────────────────────────────────────────────────────────────────");
#endif

		ProcessInfo::getInstance().outputSummary("");
	}
}

