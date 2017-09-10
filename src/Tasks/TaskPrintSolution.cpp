#include "TaskPrintSolution.h"

TaskPrintSolution::TaskPrintSolution()
{

}

TaskPrintSolution::~TaskPrintSolution()
{
}

void TaskPrintSolution::run()
{

	ProcessInfo::getInstance().stopTimer("Subproblems");

	ProcessInfo::getInstance().outputSummary(
			"                                                                                     \n");

#ifdef _WIN32
	ProcessInfo::getInstance().outputSummary("ÚÄÄÄ Solution report ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ¿");

	if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::ConstraintTolerance)
	{
		ProcessInfo::getInstance().outputSummary(
				"³ Optimal solution found to constraint tolerance "
				+ to_string(ProcessInfo::getInstance().getCurrentIteration()->maxDeviation) + " <= "
				+ to_string(Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::AbsoluteGap)
	{
		ProcessInfo::getInstance().outputSummary(
				"³ Optimal solution found to absolute gap tolerance "
				+ to_string(ProcessInfo::getInstance().getAbsoluteObjectiveGap()) + " <= "
				+ to_string(Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::RelativeGap)
	{
		ProcessInfo::getInstance().outputSummary(
				"³ Optimal solution found to relative gap tolerance "
				+ to_string(ProcessInfo::getInstance().getRelativeObjectiveGap()) + " <= "
				+ to_string(Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::TimeLimit)
	{
		ProcessInfo::getInstance().outputSummary(
				"³ Nonoptimal solution found due to time limit " + to_string(ProcessInfo::getInstance().getElapsedTime("Total"))
				+ " > " + to_string(Settings::getInstance().getDoubleSetting("TimeLimit", "Algorithm")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::IterationLimit)
	{
		ProcessInfo::getInstance().outputSummary(
				"³ Nonoptimal solution found due to iteration limit "
				+ to_string(
						Settings::getInstance().getIntSetting("IterLimitLP", "Algorithm")
						+ Settings::getInstance().getIntSetting("IterLimitMILP", "Algorithm")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::ObjectiveStagnation)
	{
		ProcessInfo::getInstance().outputSummary("³ Nonoptimal solution found due to objective function stagnation.");
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::InfeasibleProblem)
	{
		ProcessInfo::getInstance().outputSummary("³ Nonoptimal solution found since linear solver reports an infeasible problem.");
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::InteriorPointError)
	{
		ProcessInfo::getInstance().outputSummary("³ No solution found since an interior point could not be obtained.");
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::Error)
	{
		ProcessInfo::getInstance().outputSummary("³ Nonoptimal solution found since linear solver reports an error.");
	}

	auto dualBound = ProcessInfo::getInstance().getDualBound();

	auto primalBound = ProcessInfo::getInstance().getPrimalBound();

	if (ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize())
	{

		ProcessInfo::getInstance().outputSummary(
				"³ Dual bound: " + to_string(dualBound) + " Primal bound: " + to_string(primalBound)
				+ " (minimization)");
	}
	else
	{
		ProcessInfo::getInstance().outputSummary(
				"³ Dual bound: " + to_string(dualBound) + " Primal bound: " + to_string(primalBound)
				+ " (maximization)");
	}

	ProcessInfo::getInstance().outputSummary(
			"³ Relative duality gap: " + to_string(ProcessInfo::getInstance().getRelativeObjectiveGap()) + " Absolute duality gap: "
			+ to_string(ProcessInfo::getInstance().getAbsoluteObjectiveGap()));

	ProcessInfo::getInstance().outputSummary("ÃÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ´");
	ProcessInfo::getInstance().outputSummary("³ Optimal MIP problems solved:      " + to_string(ProcessInfo::getInstance().iterOptMILP));

	ProcessInfo::getInstance().outputSummary("³ Feasible MIP problems solved:     " + to_string(ProcessInfo::getInstance().iterFeasMILP));

	ProcessInfo::getInstance().outputSummary("³ Relaxed problems solved:          " + to_string(ProcessInfo::getInstance().iterLP));

	ProcessInfo::getInstance().outputSummary(
			"³ Total problems solved:            "
			+ to_string(ProcessInfo::getInstance().iterOptMILP + ProcessInfo::getInstance().iterFeasMILP + ProcessInfo::getInstance().iterLP));

	ProcessInfo::getInstance().outputSummary(
			"³ Fixed primal NLP problems solved: " + to_string(ProcessInfo::getInstance().numPrimalFixedNLPProbsSolved));

	ProcessInfo::getInstance().outputSummary("³ Total NLP problems solved:        " + to_string(ProcessInfo::getInstance().numNLPProbsSolved));

	ProcessInfo::getInstance().outputSummary("³ Function evaluations (in SHOT):   " + to_string(ProcessInfo::getInstance().numFunctionEvals));

	ProcessInfo::getInstance().outputSummary("³ Gradient evaluations (in SHOT):   " + to_string(ProcessInfo::getInstance().numGradientEvals));

	ProcessInfo::getInstance().outputSummary("ÀÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÙ");

#else
	ProcessInfo::getInstance().outputSummary(
			"┌─── Solution report ────────────────────────────────────────────────────────────┐");

	if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::ConstraintTolerance)
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Optimal solution found to constraint tolerance "
						+ to_string(ProcessInfo::getInstance().getCurrentIteration()->maxDeviation) + " <= "
						+ to_string(Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::AbsoluteGap)
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Optimal solution found to absolute gap tolerance "
						+ to_string(ProcessInfo::getInstance().getAbsoluteObjectiveGap()) + " <= "
						+ to_string(Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::RelativeGap)
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Optimal solution found to relative gap tolerance "
						+ to_string(ProcessInfo::getInstance().getRelativeObjectiveGap()) + " <= "
						+ to_string(Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::TimeLimit)
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Nonoptimal solution found due to time limit "
						+ to_string(ProcessInfo::getInstance().getElapsedTime("Total")) + " > "
						+ to_string(Settings::getInstance().getDoubleSetting("TimeLimit", "Algorithm")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::IterationLimit)
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Nonoptimal solution found due to iteration limit "
						+ to_string(
								Settings::getInstance().getIntSetting("IterLimitLP", "Algorithm")
										+ Settings::getInstance().getIntSetting("IterLimitMILP", "Algorithm")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::ObjectiveStagnation)
	{
		ProcessInfo::getInstance().outputSummary("│ Nonoptimal solution found due to objective function stagnation.");
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::InfeasibleProblem)
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Nonoptimal solution found since linear solver reports an infeasible problem.");
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::InteriorPointError)
	{
		ProcessInfo::getInstance().outputSummary("│ No solution found since an interior point could not be obtained.");
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::Error)
	{
		ProcessInfo::getInstance().outputSummary("│ Nonoptimal solution found since linear solver reports an error.");
	}

	auto dualBound = ProcessInfo::getInstance().getDualBound();

	auto primalBound = ProcessInfo::getInstance().getPrimalBound();

	if (ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize())
	{

		ProcessInfo::getInstance().outputSummary(
				"│ Dual bound: " + to_string(dualBound) + " Primal bound: " + to_string(primalBound)
						+ " (minimization)");
	}
	else
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Dual bound: " + to_string(dualBound) + " Primal bound: " + to_string(primalBound)
						+ " (maximization)");
	}

	ProcessInfo::getInstance().outputSummary(
			"│ Relative duality gap: " + to_string(ProcessInfo::getInstance().getRelativeObjectiveGap())
					+ " Absolute duality gap: " + to_string(ProcessInfo::getInstance().getAbsoluteObjectiveGap()));

	ProcessInfo::getInstance().outputSummary(
			"├────────────────────────────────────────────────────────────────────────────────┤");

	ProcessInfo::getInstance().outputSummary(
			"│ Optimal MIP problems solved:      " + to_string(ProcessInfo::getInstance().iterOptMILP));

	ProcessInfo::getInstance().outputSummary(
			"│ Feasible MIP problems solved:     " + to_string(ProcessInfo::getInstance().iterFeasMILP));

	ProcessInfo::getInstance().outputSummary(
			"│ Relaxed problems solved:          " + to_string(ProcessInfo::getInstance().iterLP));

	ProcessInfo::getInstance().outputSummary(
			"│ Total problems solved:            "
					+ to_string(
							ProcessInfo::getInstance().iterOptMILP + ProcessInfo::getInstance().iterFeasMILP
									+ ProcessInfo::getInstance().iterLP));

	ProcessInfo::getInstance().outputSummary(
			"│ Fixed primal NLP problems solved: "
					+ to_string(ProcessInfo::getInstance().numPrimalFixedNLPProbsSolved));

	ProcessInfo::getInstance().outputSummary(
			"│ Total NLP problems solved:        " + to_string(ProcessInfo::getInstance().numNLPProbsSolved));

	ProcessInfo::getInstance().outputSummary(
			"│ Function evaluations (in SHOT):   " + to_string(ProcessInfo::getInstance().numFunctionEvals));

	ProcessInfo::getInstance().outputSummary(
			"│ Gradient evaluations (in SHOT):   " + to_string(ProcessInfo::getInstance().numGradientEvals));

	ProcessInfo::getInstance().outputSummary(
			"└────────────────────────────────────────────────────────────────────────────────┘");
#endif

}

std::string TaskPrintSolution::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
