/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

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

    Output::getInstance().outputSummary(
        "                                                                                     \n");

#ifdef _WIN32
    Output::getInstance().outputSummary("ÚÄÄÄ Solution report ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ¿");

    if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::ConstraintTolerance)
    {
        Output::getInstance().outputSummary(
            "³ Optimal solution found to constraint tolerance " + UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getCurrentIteration()->maxDeviation, "%.5f") + " <= " + UtilityFunctions::toStringFormat(Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"), "%.5f"));
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::AbsoluteGap)
    {
        Output::getInstance().outputSummary(
            "³ Optimal solution found to absolute gap tolerance " + UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getAbsoluteObjectiveGap(), "%.5f") + " <= " + UtilityFunctions::toStringFormat(Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination"), "%.5f"));
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::RelativeGap)
    {
        Output::getInstance().outputSummary(
            "³ Optimal solution found to relative gap tolerance " + UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getRelativeObjectiveGap(), "%.5f") + " <= " + UtilityFunctions::toStringFormat(Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination"), "%.5f"));
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::TimeLimit)
    {
        Output::getInstance().outputSummary(
            "³ Nonoptimal solution found due to time limit " + UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getElapsedTime("Total")) + " > " + UtilityFunctions::toString(Settings::getInstance().getDoubleSetting("TimeLimit", "Termination")));
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::IterationLimit)
    {
        Output::getInstance().outputSummary(
            "³ Nonoptimal solution found due to iteration limit " + to_string(
                                                                        Settings::getInstance().getIntSetting("Relaxation.IterationLimit", "Dual") + Settings::getInstance().getIntSetting("IterationLimit", "Termination")));
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::ObjectiveStagnation)
    {
        Output::getInstance().outputSummary("³ Nonoptimal solution found due to objective function stagnation.");
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::InfeasibleProblem)
    {
        Output::getInstance().outputSummary("³ Nonoptimal solution found since linear solver reports an infeasible problem.");
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::InteriorPointError)
    {
        Output::getInstance().outputSummary("³ No solution found since an interior point could not be obtained.");
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::Error)
    {
        Output::getInstance().outputSummary("³ Nonoptimal solution found since linear solver reports an error.");
    }

    auto dualBound = ProcessInfo::getInstance().getDualBound();

    auto primalBound = ProcessInfo::getInstance().getPrimalBound();

    if (ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize())
    {

        Output::getInstance().outputSummary(
            "³ Dual bound: " + UtilityFunctions::toStringFormat(dualBound, "%.8f") + " Primal bound: " + UtilityFunctions::toStringFormat(primalBound, "%.8f") + " (minimization)");
    }
    else
    {
        Output::getInstance().outputSummary(
            "³ Dual bound: " + UtilityFunctions::toStringFormat(dualBound), "%.8f" + " Primal bound: " + UtilityFunctions::toStringFormat(primalBound, "%.8f") + " (maximization)");
    }

    Output::getInstance().outputSummary(
        "³ Relative duality gap: " + UtilityFunctions::toString(ProcessInfo::getInstance().getRelativeObjectiveGap()) + " Absolute duality gap: " + UtilityFunctions::toString(ProcessInfo::getInstance().getAbsoluteObjectiveGap()));

    Output::getInstance().outputSummary("ÃÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ´");
    Output::getInstance().outputSummary("³ Optimal MIP problems solved:      " + to_string(ProcessInfo::getInstance().iterOptMILP));

    Output::getInstance().outputSummary("³ Feasible MIP problems solved:     " + to_string(ProcessInfo::getInstance().iterFeasMILP));

    Output::getInstance().outputSummary("³ Relaxed problems solved:          " + to_string(ProcessInfo::getInstance().iterLP));

    Output::getInstance().outputSummary(
        "³ Total problems solved:            " + to_string(ProcessInfo::getInstance().iterOptMILP + ProcessInfo::getInstance().iterFeasMILP + ProcessInfo::getInstance().iterLP));

    Output::getInstance().outputSummary(
        "³ Fixed primal NLP problems solved: " + to_string(ProcessInfo::getInstance().numPrimalFixedNLPProbsSolved));

    Output::getInstance().outputSummary("³ Total NLP problems solved:        " + to_string(ProcessInfo::getInstance().numNLPProbsSolved));

    Output::getInstance().outputSummary("³ Function evaluations (in SHOT):   " + to_string(ProcessInfo::getInstance().numFunctionEvals));

    Output::getInstance().outputSummary("³ Gradient evaluations (in SHOT):   " + to_string(ProcessInfo::getInstance().numGradientEvals));

    Output::getInstance().outputSummary("ÀÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÙ");

#else
    Output::getInstance().outputSummary(
        "┌─── Solution report ────────────────────────────────────────────────────────────┐");

    if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::ConstraintTolerance)
    {
        Output::getInstance().outputSummary(
            "│ Optimal solution found to constraint tolerance " + UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getCurrentIteration()->maxDeviation, "%.5f") + " <= " + UtilityFunctions::toStringFormat(Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"), "%.5f"));
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::AbsoluteGap)
    {
        Output::getInstance().outputSummary(
            "│ Optimal solution found to absolute gap tolerance " + UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getAbsoluteObjectiveGap(), "%.5f") + " <= " + UtilityFunctions::toStringFormat(Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination"), "%.5f"));
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::RelativeGap)
    {
        Output::getInstance().outputSummary(
            "│ Optimal solution found to relative gap tolerance " + UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getRelativeObjectiveGap(), "%.5f") + " <= " + UtilityFunctions::toStringFormat(Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination"), "%.5f"));
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::TimeLimit)
    {
        Output::getInstance().outputSummary(
            "│ Nonoptimal solution found due to time limit " + UtilityFunctions::toString(ProcessInfo::getInstance().getElapsedTime("Total")) + " > " + UtilityFunctions::toString(Settings::getInstance().getDoubleSetting("TimeLimit", "Termination")));
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::IterationLimit)
    {
        Output::getInstance().outputSummary(
            "│ Nonoptimal solution found due to iteration limit " + to_string(
                                                                        Settings::getInstance().getIntSetting("Relaxation.IterationLimit", "Dual") + Settings::getInstance().getIntSetting("IterationLimit", "Termination")));
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::ObjectiveStagnation)
    {
        Output::getInstance().outputSummary("│ Nonoptimal solution found due to objective function stagnation.");
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::InfeasibleProblem)
    {
        Output::getInstance().outputSummary(
            "│ Nonoptimal solution found since linear solver reports an infeasible problem.");
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::InteriorPointError)
    {
        Output::getInstance().outputSummary("│ No solution found since an interior point could not be obtained.");
    }
    else if (ProcessInfo::getInstance().terminationReason == E_TerminationReason::Error)
    {
        Output::getInstance().outputSummary("│ Nonoptimal solution found since linear solver reports an error.");
    }

    auto dualBound = ProcessInfo::getInstance().getDualBound();

    auto primalBound = ProcessInfo::getInstance().getPrimalBound();

    if (ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize())
    {
        Output::getInstance().outputSummary(
            "│ Dual bound: " + UtilityFunctions::toStringFormat(dualBound, "%.8f") + " Primal bound: " + UtilityFunctions::toStringFormat(primalBound, "%.8f") + " (minimization)");
    }
    else
    {
        Output::getInstance().outputSummary(
            "│ Dual bound: " + UtilityFunctions::toStringFormat(dualBound, "%.8f") + " Primal bound: " + UtilityFunctions::toStringFormat(primalBound, "%.8f") + " (maximization)");
    }

    Output::getInstance().outputSummary(
        "│ Relative duality gap: " + UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getRelativeObjectiveGap(), "%.8f") + " Absolute duality gap: " + UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getAbsoluteObjectiveGap(), "%.8f"));

    Output::getInstance().outputSummary(
        "├────────────────────────────────────────────────────────────────────────────────┤");

    Output::getInstance().outputSummary(
        "│ Optimal MIP problems solved:      " + to_string(ProcessInfo::getInstance().iterOptMILP));

    Output::getInstance().outputSummary(
        "│ Feasible MIP problems solved:     " + to_string(ProcessInfo::getInstance().iterFeasMILP));

    Output::getInstance().outputSummary(
        "│ Relaxed problems solved:          " + to_string(ProcessInfo::getInstance().iterLP));

    Output::getInstance().outputSummary(
        "│ Total problems solved:            " + to_string(
                                                     ProcessInfo::getInstance().iterOptMILP + ProcessInfo::getInstance().iterFeasMILP + ProcessInfo::getInstance().iterLP));

    Output::getInstance().outputSummary(
        "│ Fixed primal NLP problems solved: " + to_string(ProcessInfo::getInstance().numPrimalFixedNLPProbsSolved));

    Output::getInstance().outputSummary(
        "│ Total NLP problems solved:        " + to_string(ProcessInfo::getInstance().numNLPProbsSolved));

    Output::getInstance().outputSummary(
        "│ Function evaluations (in SHOT):   " + to_string(ProcessInfo::getInstance().numFunctionEvals));

    Output::getInstance().outputSummary(
        "│ Gradient evaluations (in SHOT):   " + to_string(ProcessInfo::getInstance().numGradientEvals));

    Output::getInstance().outputSummary(
        "└────────────────────────────────────────────────────────────────────────────────┘");
#endif
}

std::string TaskPrintSolution::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
