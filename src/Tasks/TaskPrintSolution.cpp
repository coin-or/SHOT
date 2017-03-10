#include "TaskPrintSolution.h"

TaskPrintSolution::TaskPrintSolution()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskPrintSolution::~TaskPrintSolution()
{
}

void TaskPrintSolution::run()
{

	processInfo->stopTimer("Subproblems");

	//auto tmpSol = processInfo->MILPSolver->getObjectiveValue();
	processInfo->outputSummary("\n"
			"┌─── Solution report ────────────────────────────────────────────────────────────┐");

	if (processInfo->terminationReason == E_TerminationReason::ConstraintTolerance)
	{
		processInfo->outputSummary(
				"│ Optimal solution found to constraint tolerance "
						+ to_string(processInfo->getCurrentIteration()->maxDeviation) + " <= "
						+ to_string(settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm")));
	}
	else if (processInfo->terminationReason == E_TerminationReason::AbsoluteGap)
	{
		processInfo->outputSummary(
				"│ Optimal solution found to absolute gap tolerance "
						+ to_string(processInfo->getAbsoluteObjectiveGap()) + " <= "
						+ to_string(settings->getDoubleSetting("GapTermTolAbsolute", "Algorithm")));
	}
	else if (processInfo->terminationReason == E_TerminationReason::RelativeGap)
	{
		processInfo->outputSummary(
				"│ Optimal solution found to relative gap tolerance "
						+ to_string(processInfo->getRelativeObjectiveGap()) + " <= "
						+ to_string(settings->getDoubleSetting("GapTermTolRelative", "Algorithm")));
	}
	else if (processInfo->terminationReason == E_TerminationReason::TimeLimit)
	{
		processInfo->outputSummary(
				"│ Nonoptimal solution found due to time limit " + to_string(processInfo->getElapsedTime("Total"))
						+ " > " + to_string(settings->getDoubleSetting("TimeLimit", "Algorithm")));
	}
	else if (processInfo->terminationReason == E_TerminationReason::IterationLimit)
	{
		processInfo->outputSummary(
				"│ Nonoptimal solution found due to iteration limit "
						+ to_string(
								settings->getIntSetting("IterLimitLP", "Algorithm")
										+ settings->getIntSetting("IterLimitMILP", "Algorithm")));
	}
	else if (processInfo->terminationReason == E_TerminationReason::ObjectiveStagnation)
	{
		processInfo->outputSummary("│ Nonoptimal solution found due to objective function stagnation.");
	}
	else if (processInfo->terminationReason == E_TerminationReason::InfeasibleProblem)
	{
		processInfo->outputSummary("│ Nonoptimal solution found since linear solver reports an infeasible problem.");
	}
	else if (processInfo->terminationReason == E_TerminationReason::InteriorPointError)
	{
		processInfo->outputSummary("│ No solution found since an interior point could not be obtained.");
	}
	else if (processInfo->terminationReason == E_TerminationReason::Error)
	{
		processInfo->outputSummary("│ Nonoptimal solution found since linear solver reports an error.");
	}

	auto dualBound = processInfo->getDualBound();

	auto primalBound = processInfo->getPrimalBound();

	if (processInfo->originalProblem->isTypeOfObjectiveMinimize())
	{

		processInfo->outputSummary(
				"│ Dual bound: " + to_string(dualBound) + " Primal bound: " + to_string(primalBound)
						+ " (minimization)");
	}
	else
	{
		processInfo->outputSummary(
				"│ Dual bound: " + to_string(dualBound) + " Primal bound: " + to_string(primalBound)
						+ " (maximization)");
	}

	processInfo->outputSummary(
			"│ Relative duality gap: " + to_string(processInfo->getRelativeObjectiveGap()) + " Absolute duality gap: "
					+ to_string(processInfo->getAbsoluteObjectiveGap()));

	processInfo->outputSummary("├────────────────────────────────────────────────────────────────────────────────┤");

	processInfo->outputSummary("│ Optimal MIP problems solved:      " + to_string(processInfo->iterOptMILP));

	processInfo->outputSummary("│ Feasible MIP problems solved:     " + to_string(processInfo->iterFeasMILP));

	processInfo->outputSummary("│ Relaxed problems solved:          " + to_string(processInfo->iterLP));

	processInfo->outputSummary(
			"│ Total problems solved:            "
					+ to_string(processInfo->iterOptMILP + processInfo->iterFeasMILP + processInfo->iterLP));

	processInfo->outputSummary(
			"│ Fixed primal NLP problems solved: " + to_string(processInfo->numPrimalFixedNLPProbsSolved));

	processInfo->outputSummary("│ Total NLP problems solved:        " + to_string(processInfo->numNLPProbsSolved));

	processInfo->outputSummary("│ Function evaluations (in SHOT):   " + to_string(processInfo->numFunctionEvals));

	processInfo->outputSummary("│ Gradient evaluations (in SHOT):   " + to_string(processInfo->numGradientEvals));

	processInfo->outputSummary("└────────────────────────────────────────────────────────────────────────────────┘");

}

std::string TaskPrintSolution::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
