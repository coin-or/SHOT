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
				+ UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getCurrentIteration()->maxDeviation, "%.5f") + " <= "
				+ UtilityFunctions::toStringFormat(Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"), "%.5f"));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::AbsoluteGap)
	{
		ProcessInfo::getInstance().outputSummary(
				"³ Optimal solution found to absolute gap tolerance "
				+ UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getAbsoluteObjectiveGap(), "%.5f") + " <= "
				+ UtilityFunctions::toStringFormat(Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination"), "%.5f"));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::RelativeGap)
	{
		ProcessInfo::getInstance().outputSummary(
				"³ Optimal solution found to relative gap tolerance "
				+ UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getRelativeObjectiveGap(), "%.5f") + " <= "
				+ UtilityFunctions::toStringFormat(Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination"), "%.5f"));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::TimeLimit)
	{
		ProcessInfo::getInstance().outputSummary(
				"³ Nonoptimal solution found due to time limit " + UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getElapsedTime("Total"))
				+ " > " + UtilityFunctions::toString(Settings::getInstance().getDoubleSetting("TimeLimit", "Termination")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::IterationLimit)
	{
		ProcessInfo::getInstance().outputSummary(
				"³ Nonoptimal solution found due to iteration limit "
				+ to_string(
						Settings::getInstance().getIntSetting("Relaxation.IterationLimit", "Dual")
						+ Settings::getInstance().getIntSetting("IterationLimit", "Termination")));
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
				"³ Dual bound: " + UtilityFunctions::toStringFormat(dualBound, "%.8f") + " Primal bound: " + UtilityFunctions::toStringFormat(primalBound, "%.8f")
				+ " (minimization)");
	}
	else
	{
		ProcessInfo::getInstance().outputSummary(
				"³ Dual bound: " + UtilityFunctions::toStringFormat(dualBound), "%.8f" + " Primal bound: " + UtilityFunctions::toStringFormat(primalBound, "%.8f")
				+ " (maximization)");
	}

	ProcessInfo::getInstance().outputSummary(
			"³ Relative duality gap: " + UtilityFunctions::toString(ProcessInfo::getInstance().getRelativeObjectiveGap()) + " Absolute duality gap: "
			+ UtilityFunctions::toString(ProcessInfo::getInstance().getAbsoluteObjectiveGap()));

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
						+ UtilityFunctions::toStringFormat(
								ProcessInfo::getInstance().getCurrentIteration()->maxDeviation, "%.5f") + " <= "
						+ UtilityFunctions::toStringFormat(
								Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"), "%.5f"));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::AbsoluteGap)
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Optimal solution found to absolute gap tolerance "
						+ UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getAbsoluteObjectiveGap(), "%.5f")
						+ " <= "
						+ UtilityFunctions::toStringFormat(
								Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination"), "%.5f"));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::RelativeGap)
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Optimal solution found to relative gap tolerance "
						+ UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getRelativeObjectiveGap(), "%.5f")
						+ " <= "
						+ UtilityFunctions::toStringFormat(
								Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination"), "%.5f"));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::TimeLimit)
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Nonoptimal solution found due to time limit "
						+ UtilityFunctions::toString(ProcessInfo::getInstance().getElapsedTime("Total")) + " > "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("TimeLimit", "Termination")));
	}
	else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::IterationLimit)
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Nonoptimal solution found due to iteration limit "
						+ to_string(
								Settings::getInstance().getIntSetting("Relaxation.IterationLimit", "Dual")
										+ Settings::getInstance().getIntSetting("IterationLimit", "Termination")));
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
				"│ Dual bound: " + UtilityFunctions::toStringFormat(dualBound, "%.8f") + " Primal bound: "
						+ UtilityFunctions::toStringFormat(primalBound, "%.8f") + " (minimization)");
	}
	else
	{
		ProcessInfo::getInstance().outputSummary(
				"│ Dual bound: " + UtilityFunctions::toStringFormat(dualBound, "%.8f") + " Primal bound: "
						+ UtilityFunctions::toStringFormat(primalBound, "%.8f") + " (maximization)");
	}

	ProcessInfo::getInstance().outputSummary(
			"│ Relative duality gap: "
					+ UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getRelativeObjectiveGap(), "%.8f")
					+ " Absolute duality gap: "
					+ UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getAbsoluteObjectiveGap(), "%.8f"));

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
