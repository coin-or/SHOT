/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "Report.h"

namespace SHOT
{

Report::Report(EnvironmentPtr envPtr) : env(envPtr)
{
}

Report::~Report()
{
}

void Report::outputIterationDetail(int iterationNumber,
                                   std::string iterationDesc,
                                   double totalTime,
                                   int dualCutsAdded,
                                   int dualCutsTotal,
                                   double dualObjectiveValue,
                                   double primalObjectiveValue,
                                   double absoluteObjectiveGap,
                                   double relativeObjectiveGap,
                                   double currentObjectiveValue,
                                   int maxConstraintIndex,
                                   double maxConstraintError,
                                   E_IterationLineType lineType)
{
    try
    {
        bool printLine = false;

        if (dualObjectiveValue != lastDualObjectiveValue)
        {
            lastDualObjectiveValue = dualObjectiveValue;
            printLine = true;
        }

        if (primalObjectiveValue != lastPrimalObjectiveValue)
        {
            lastPrimalObjectiveValue = primalObjectiveValue;
            printLine = true;
        }

        if (iterationsWithoutPrintoutCounter > 100 || env->process->getElapsedTime("Total") - lastIterationOutputTimeStamp > 2)
        {
            printLine = true;
        }

        switch (static_cast<ES_IterationOutputDetail>(env->settings->getIntSetting("Console.Iteration.Detail", "Output")))
        {
        case ES_IterationOutputDetail::Full:
            printLine = true;
            break;

        case ES_IterationOutputDetail::ObjectiveGapUpdatesAndNLPCalls:
            if (lineType == E_IterationLineType::PrimalNLP)
                printLine = true;
            break;

        default:
            break;
        }

        if (!printLine)
        {
            iterationsWithoutPrintoutCounter++;
            return;
        }

        if (!firstIterationHeaderPrinted)
        {
            this->outputIterationDetailHeader();
            firstIterationHeaderPrinted = true;
        }

        if (iterationPrintoutsSinceLastHeader > 75)
        {
            this->outputIterationDetailHeader();
        }

        iterationsWithoutPrintoutCounter = 0;
        iterationPrintoutsSinceLastHeader++;
        lastIterationOutputTimeStamp = env->process->getElapsedTime("Total");

        std::string combDualCuts = "";

        if (dualCutsAdded > 0)
        {
            combDualCuts = (boost::format("%|4i| | %|-6i|") % dualCutsAdded % dualCutsTotal).str();
        }

        std::string combObjectiveValue = (boost::format("%|12s| | %|-12s|") % UtilityFunctions::toStringFormat(dualObjectiveValue, "%#g") % UtilityFunctions::toStringFormat(primalObjectiveValue, "%#g")).str();

        std::string combObjectiveGap = (boost::format("%|8s| | %|-8s|") % UtilityFunctions::toStringFormat(absoluteObjectiveGap, "%#.1e") % UtilityFunctions::toStringFormat(relativeObjectiveGap, "%#.1e")).str();

        std::string combCurrSol;

        if (UtilityFunctions::isnan(currentObjectiveValue))
        {
            combCurrSol = "            inf.";
        }
        else
        {
            combCurrSol = (boost::format("%|#12g| | %|+.1e| (%|i|)") % currentObjectiveValue % maxConstraintError % maxConstraintIndex).str();
        }

        auto tmpLine = boost::format("%|6i|: %|-10s|%|#=10.2f|%|13s|%|27s|%|19s|%|-32s|") % iterationNumber % iterationDesc % totalTime % combDualCuts % combObjectiveValue % combObjectiveGap % combCurrSol;

        env->output->outputSummary(tmpLine.str());

        std::stringstream nodes;

        nodes << "        Explored nodes: ";

        if (env->process->getCurrentIteration()->numberOfExploredNodes > 0)
        {
            nodes << " +" << env->process->getCurrentIteration()->numberOfExploredNodes
                  << " = ";
        }

        nodes
            << env->solutionStatistics.numberOfExploredNodes << ".";

        if (env->process->getCurrentIteration()->numberOfOpenNodes > 0)
        {
            nodes << " Open nodes: " << env->process->getCurrentIteration()->numberOfOpenNodes << ".";
        }

        nodes << "\r\n";

        env->output->outputInfo(nodes.str());
    }
    catch (...)
    {
        env->output->outputError("ERROR, cannot write iteration solution report!");
    }
}

void Report::outputIterationDetailMinimax(int iterationNumber,
                                          std::string iterationDesc,
                                          double totalTime,
                                          int dualCutsAdded,
                                          int dualCutsTotal,
                                          double dualObjectiveValue,
                                          double primalObjectiveValue,
                                          double absoluteObjectiveGap,
                                          double relativeObjectiveGap)
{
    try
    {
        std::string combDualCuts = "";

        if (dualCutsAdded > 0)
        {
            combDualCuts = (boost::format("%|4i| | %|-6i|") % dualCutsAdded % dualCutsTotal).str();
        }

        if (dualObjectiveValue != lastDualObjectiveValue)
        {
            lastDualObjectiveValue = dualObjectiveValue;
        }

        if (primalObjectiveValue != lastPrimalObjectiveValue)
        {
            lastPrimalObjectiveValue = primalObjectiveValue;
        }

        std::string combObjectiveValue = (boost::format("%|12s| | %|-12s|") % UtilityFunctions::toStringFormat(dualObjectiveValue, "%#g") % UtilityFunctions::toStringFormat(primalObjectiveValue, "%#g")).str();

        if (absoluteObjectiveGap != lastAbsoluteObjectiveGap)
        {
            lastAbsoluteObjectiveGap = absoluteObjectiveGap;
        }

        if (relativeObjectiveGap != lastRelativeObjectiveGap)
        {
            lastRelativeObjectiveGap = relativeObjectiveGap;
        }

        std::string combObjectiveGap = (boost::format("%|8s| | %|-8s|") % UtilityFunctions::toStringFormat(absoluteObjectiveGap, "%#.1e") % UtilityFunctions::toStringFormat(relativeObjectiveGap, "%#.1e")).str();

        auto tmpLine = boost::format("%|6i|: %|-10s|%|#=10.2f|%|13s|%|27s|%|19s|") % iterationNumber % iterationDesc % totalTime % combDualCuts % combObjectiveValue % combObjectiveGap;

        env->output->outputSummary(tmpLine.str());
    }
    catch (...)
    {
        env->output->outputError("ERROR, cannot write iteration solution report!");
    }
}

void Report::outputIterationDetailHeader()
{
    std::stringstream header;

    header << "\r\n";
    header << "                                                                                     \n";

    header << "    Iteration     │  Time  │  Dual cuts  │     Objective value     │   Objective gap   │     Current solution\r\n";
    header << "     #: type      │  tot.  │   + | tot.  │       dual | primal     │    abs. | rel.    │    obj.fn. | max.err.\r\n";
    header << "╶─────────────────┴────────┴─────────────┴─────────────────────────┴───────────────────┴───────────────────────────╴\r\n";
    header << "\r\n";

    env->output->outputSummary(header.str());
    iterationPrintoutsSinceLastHeader = 0;
}

void Report::outputIterationDetailHeaderMinimax()
{
    std::stringstream header;
    header << "                                                                                     \n";

    header << "    Iteration     │  Time  │    Cuts     │     Objective value     │  Objective diff.   \r\n";
    header << "     #: type      │  tot.  │   + | tot.  │    problem | line srch  │    abs. | rel.    \r\n";
    header << "╶─────────────────┴────────┴─────────────┴─────────────────────────┴──────────────────╴\r\n";

    env->output->outputSummary(header.str());
}

void Report::outputSolverHeader()
{
    std::stringstream header;

    header << "\r\n";
    header << "╶ Supporting Hyperplane Optimization Toolkit (SHOT) ──────────────────────────────────────────────────────────────────╴\r\n";
    header << "\r\n";

    header << "  Andreas Lundell, Jan Kronqvist, Tapio Westerlund\r\n";
    header << "  Faculty of Science and Engineering, Åbo Akademi University\r\n";

    header << "\r\n";
    header << "  Version: ";
    header << SHOT_VERSION_MAJOR;
    header << ".";
    header << SHOT_VERSION_MINOR;

    if (SHOT_VERSION_PATCH != 0)
    {
        header << ".";
        header << SHOT_VERSION_PATCH;
    }

    header << ", released ";
    header << __DATE__;
    header << "\r\n";

    env->output->outputSummary(header.str());
}

void Report::outputOptionsReport()
{
    std::stringstream report;

    report << "\r\n";
    report << "╶ Options ────────────────────────────────────────────────────────────────────────────────────────────────────────────╴\r\n";
    report << "\r\n";

    std::string optionsFile = env->settings->getStringSetting("OptionsFile", "Input");

    if (optionsFile == "")
    {
        report << " No options file specified.\r\n";
    }
    else
    {
        report << " Options read from file:     ";
        report << optionsFile;
        report << "\r\n";
    }

    report << "\r\n";

    std::string nonDefaultOptions = env->settings->getUpdatedSettingsAsString();

    if (nonDefaultOptions != "")
    {
        report << " Nondefault options used:\r\n";
        report << "\r\n";
        report << nonDefaultOptions;

        report << "\r\n";
    }

    std::string cutAlgorithm, dualSolver;
    bool useSingleTree = (static_cast<ES_TreeStrategy>(env->settings->getIntSetting("TreeStrategy", "Dual")) == ES_TreeStrategy::SingleTree);

    if (static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
    {
        cutAlgorithm = "ESH";
    }
    else
    {
        cutAlgorithm = "ECP";
    }

    auto solver = static_cast<ES_MIPSolver>(env->settings->getIntSetting("MIP.Solver", "Dual"));

    if (useSingleTree)
    {
#ifdef HAS_CPLEX
        if (solver == ES_MIPSolver::Cplex)
        {
#ifdef HAS_CPLEX_NEW_CALLBACK
            if (env->settings->getBoolSetting("Cplex.UseNewCallbackType", "Subsolver"))
            {
                dualSolver = "CPLEX with new callback functionality";
            }
            else
#endif
            {
                dualSolver = "CPLEX with old callback functionality";
            }
        }
#endif

#ifdef HAS_GUROBI
        if (solver == ES_MIPSolver::Gurobi)
        {
            dualSolver = "Gurobi";
        }
#endif

        if (solver == ES_MIPSolver::Cbc)
        {
            dualSolver = "Cbc";
        }
    }
    else
    {

#ifdef HAS_CPLEX
        if (solver == ES_MIPSolver::Cplex)
        {
            dualSolver = "CPLEX";
        }
#endif

#ifdef HAS_GUROBI
        if (solver == ES_MIPSolver::Gurobi)
        {
            dualSolver = "Gurobi";
        }
#endif
        if (solver == ES_MIPSolver::Cbc)
        {
            dualSolver = "Cbc";
        }
    }

    switch (static_cast<E_SolutionStrategy>(env->process->usedSolutionStrategy))
    {
    case (E_SolutionStrategy::SingleTree):
        report << " Dual strategy:              Single-tree\r\n";
        report << "  - cut algorithm:           " << cutAlgorithm << "\r\n";
        break;
    case (E_SolutionStrategy::MultiTree):
        report << " Dual strategy:              Multi-tree\r\n";
        report << "  - cut algorithm:           " << cutAlgorithm << "\r\n";
        break;
    case (E_SolutionStrategy::NLP):
        report << " Dual strategy:              NLP version\r\n";
        report << "  - cut algorithm:           " << cutAlgorithm << "\r\n";
        break;
    case (E_SolutionStrategy::MIQP):
        report << " Dual strategy:              MIQP version\r\n";
        break;
    case (E_SolutionStrategy::MIQCQP):
        report << " Dual strategy:              MIQCQP version\r\n";
        break;
    default:
        break;
    }

    report << "  - solver:                  " << dualSolver << "\r\n";

    report << "\r\n";

    report << " Primal NLP solver:          ";

    switch (static_cast<ES_PrimalNLPSolver>(env->process->usedPrimalNLPSolver))
    {
    case (ES_PrimalNLPSolver::None):
        report << "none";
        break;
    case (ES_PrimalNLPSolver::CuttingPlane):
        report << "cutting plane";
        break;
    case (ES_PrimalNLPSolver::GAMS):
        report << "GAMS (";
        report << env->settings->getStringSetting("GAMS.NLP.Solver", "Subsolver");
        report << ")\r\n";
        break;
    case (ES_PrimalNLPSolver::Ipopt):
        report << "Ipopt (";

        switch (static_cast<ES_IpoptSolver>(env->settings->getIntSetting("Ipopt.LinearSolver", "Subsolver")))
        {
        case (ES_IpoptSolver::ma27):
            report << "ma27";
            break;

        case (ES_IpoptSolver::ma57):
            report << "ma57";
            break;

        case (ES_IpoptSolver::ma86):
            report << "ma86";
            break;

        case (ES_IpoptSolver::ma97):
            report << "ma97";
            break;

        case (ES_IpoptSolver::mumps):
            report << "mumps";
            break;
        default:
            report << "mumps";
        }

        report << ")\r\n";
        break;
    default:
        report << "none";
        break;
    }

    report << "\r\n";

    if (env->settings->getBoolSetting("Debug.Enable", "Output"))
    {
        report << " Debug directory:            ";
        report << env->settings->getStringSetting("Debug.Path", "Output");
        report << "\r\n";
    }

    env->output->outputSummary(report.str());
}

void Report::outputProblemInstanceReport()
{
    std::stringstream report;

    report << "\r\n";
    report << "╶ Problem instance ───────────────────────────────────────────────────────────────────────────────────────────────────╴\r\n";
    report << "\r\n";

    std::string problemFile = env->settings->getStringSetting("ProblemFile", "Input");

    report << " Problem read from file:     " << problemFile;
    report << "\r\n";

    report << " Objective function type:    ";

    switch (static_cast<E_ObjectiveFunctionClassification>(env->problem->objectiveFunction->properties.classification))
    {
    case (E_ObjectiveFunctionClassification::Linear):
        report << "linear";
        break;

    case (E_ObjectiveFunctionClassification::Quadratic):
        report << "quadratic";
        break;

    case (E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear):
        report << "quadratic but considered as nonlinear";
        break;

    case (E_ObjectiveFunctionClassification::Signomial):
        report << "signomial";
        break;

    case (E_ObjectiveFunctionClassification::Nonlinear):
        report << "nonlinear";
        break;

    case (E_ObjectiveFunctionClassification::GeneralizedSignomial):
        report << "generalized signomial";
        break;

    case (E_ObjectiveFunctionClassification::Nonalgebraic):
        report << "nonalgebraic";
        break;

    default:
        report << "unknown";
        break;
    }

    report << "\r\n";
    report << "\r\n";

    report << " Number of constraints:      " << env->problem->properties.numberOfNumericConstraints << "\r\n";

    if (env->problem->properties.numberOfLinearConstraints > 0)
        report << "  - linear:                  " << env->problem->properties.numberOfLinearConstraints << "\r\n";
    if (env->problem->properties.numberOfNonlinearConstraints > 0)
        report << "  - nonlinear:               " << env->problem->properties.numberOfNonlinearConstraints << "\r\n";
    if (env->problem->properties.numberOfQuadraticConstraints > 0)
    {
        report << "  - quadratic:               " << env->problem->properties.numberOfQuadraticConstraints;

        //if (env->problem->properties->quadraticTermsReformulatedAsNonlinear)
        //report << " (considered as nonlinear)\r\n";
        //else
        report << "\r\n";
    }

    report << "\r\n";

    report << " Number of variables:        " << env->problem->properties.numberOfVariables << "\r\n";
    if (env->problem->properties.numberOfRealVariables > 0)
        report << "  - real:                    " << env->problem->properties.numberOfRealVariables << "\r\n";
    if (env->problem->properties.numberOfBinaryVariables > 0)
        report << "  - binary:                  " << env->problem->properties.numberOfBinaryVariables << "\r\n";
    if (env->problem->properties.numberOfIntegerVariables > 0)
        report << "  - integer:                 " << env->problem->properties.numberOfIntegerVariables << "\r\n";
    if (env->problem->properties.numberOfSemicontinuousVariables > 0)
        report << "  - semicontinuous:          " << env->problem->properties.numberOfSemicontinuousVariables << "\r\n";

    env->output->outputSummary(report.str());
}

void Report::outputSolutionReport()
{
    std::stringstream report;

    report << "\r\n\r\n";
    report << "╶ Solution report ────────────────────────────────────────────────────────────────────────────────────────────────────╴\r\n";
    report << "\r\n";

    auto terminationReason = env->process->terminationReason;

    bool primalSolutionFound = (env->process->primalSolutions.size() > 0);

    if (terminationReason == E_TerminationReason::AbsoluteGap ||
        terminationReason == E_TerminationReason::RelativeGap)
    {
        report << " Optimal primal solution found to given tolerances.\r\n";
    }
    else if (terminationReason == E_TerminationReason::InfeasibleProblem)
    {
        report << " No solution found since problem is infeasible.\r\n";
    }
    else if (terminationReason == E_TerminationReason::ConstraintTolerance ||
             terminationReason == E_TerminationReason::ObjectiveGapNotReached ||
             terminationReason == E_TerminationReason::ObjectiveStagnation ||
             terminationReason == E_TerminationReason::IterationLimit ||
             terminationReason == E_TerminationReason::TimeLimit)
    {
        if (primalSolutionFound)
            report << " Feasible primal solution found. Can not guarantee optimality to the given termination criteria.\r\n";
        else
            report << " No feasible primal solution found. Try modifying the termination criteria.\r\n";
    }
    else if (terminationReason == E_TerminationReason::UnboundedProblem)
    {
        report << " No solution found since problem is unbounded.\r\n";
    }
    else if (terminationReason == E_TerminationReason::NumericIssues)
    {
        if (primalSolutionFound)
            report << " Feasible primal solution found. Can not guarantee optimality due to numeric issues.\r\n";
        else
            report << " No feasible primal solution found due to numeric issues.\r\n";
    }
    else if (terminationReason == E_TerminationReason::UserAbort)
    {
        if (primalSolutionFound)
            report << " Feasible primal solution found. Can not guarantee optimality since solution process was aborted.\r\n";
        else
            report << " No feasible primal solution found since solution process was aborted.\r\n";
    }
    else if (terminationReason == E_TerminationReason::Error)
    {
        if (primalSolutionFound)
            report << " Feasible primal solution found. Can not guarantee optimality since an error occured.\r\n";
        else
            report << " No feasible primal solution found since an error occured.\r\n";
    }
    else
    {
        if (primalSolutionFound)
            report << " Feasible primal solution found. Can not guarantee optimality since an error occured.\r\n";
        else
            report << " No feasible primal solution found since an error occured.\r\n";
    }

    report << "\r\n";

    report << " Objective bound [dual, primal]:                 ";
    report << "[" << UtilityFunctions::toStringFormat(env->process->getDualBound(), "%g") << ", ";
    report << UtilityFunctions::toStringFormat(env->process->getPrimalBound(), "%g") << "]\r\n";
    report << " Objective gap absolute / relative:              ";
    report << "" << UtilityFunctions::toStringFormat(env->process->getAbsoluteObjectiveGap(), "%g") << " / ";
    report << UtilityFunctions::toStringFormat(env->process->getRelativeObjectiveGap(), "%g") << "\r\n";
    report << "\r\n";

    std::stringstream fulfilled;
    std::stringstream unfulfilled;

    if (env->process->isAbsoluteObjectiveGapToleranceMet())
    {
        fulfilled << "  - absolute objective gap tolerance             ";
        fulfilled << env->process->getAbsoluteObjectiveGap() << " <= ";
        fulfilled << env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination") << "\r\n";
    }
    else
    {
        unfulfilled << "  - absolute objective gap tolerance             ";
        unfulfilled << env->process->getAbsoluteObjectiveGap() << " > ";
        unfulfilled << env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination") << "\r\n";
    }

    if (env->process->isRelativeObjectiveGapToleranceMet())
    {
        fulfilled << "  - relative objective gap tolerance             ";
        fulfilled << env->process->getRelativeObjectiveGap() << " <= ";
        fulfilled << env->settings->getDoubleSetting("ObjectiveGap.Relative", "Termination") << "\r\n";
    }
    else
    {
        unfulfilled << "  - relative objective gap tolerance             ";
        unfulfilled << env->process->getRelativeObjectiveGap() << " > ";
        unfulfilled << env->settings->getDoubleSetting("ObjectiveGap.Relative", "Termination") << "\r\n";
    }

    if (static_cast<ES_TreeStrategy>(env->settings->getIntSetting("TreeStrategy", "Dual")) != ES_TreeStrategy::SingleTree)
    {
        if (env->process->getCurrentIteration()->maxDeviation <= env->settings->getDoubleSetting("ConstraintTolerance", "Termination"))
        {
            fulfilled << "  - maximal constraint tolerance                 ";
            fulfilled << env->process->getCurrentIteration()->maxDeviation << " <= ";
            fulfilled << env->settings->getDoubleSetting("ConstraintTolerance", "Termination") << "\r\n";
        }
        else
        {
            unfulfilled << "  - maximal constraint tolerance                 ";
            unfulfilled << env->process->getCurrentIteration()->maxDeviation << " > ";
            unfulfilled << env->settings->getDoubleSetting("ConstraintTolerance", "Termination") << "\r\n";
        }
    }

    int iterLim = env->settings->getIntSetting("Relaxation.IterationLimit", "Dual") + env->settings->getIntSetting("IterationLimit", "Termination");

    if (env->process->getCurrentIteration()->iterationNumber > iterLim)
    {
        fulfilled << "  - iteration limit                              ";
        fulfilled << env->process->getCurrentIteration()->iterationNumber << " > ";
        fulfilled << iterLim << "\r\n";
    }
    else
    {
        unfulfilled << "  - iteration limit                              ";
        unfulfilled << env->process->getCurrentIteration()->iterationNumber << " <= ";
        unfulfilled << iterLim << "\r\n";
    }

    if (env->process->getElapsedTime("Total") > env->settings->getDoubleSetting("TimeLimit", "Termination"))
    {
        fulfilled << "  - solution time limit (s)                      ";
        fulfilled << env->process->getElapsedTime("Total") << " > ";
        fulfilled << env->settings->getDoubleSetting("TimeLimit", "Termination") << "\r\n";
    }
    else
    {
        unfulfilled << "  - solution time limit (s)                      ";
        unfulfilled << env->process->getElapsedTime("Total") << " <= ";
        unfulfilled << env->settings->getDoubleSetting("TimeLimit", "Termination") << "\r\n";
    }

    report << " Fulfilled termination criteria: \r\n";
    report << fulfilled.str();
    report << "\r\n";

    report << " Unfulfilled termination criteria: \r\n";
    report << unfulfilled.str();
    report << "\r\n";

    report << " Dual problems solved in main step:              ";
    report << env->solutionStatistics.getNumberOfTotalDualProblems() << "\r\n";

    if (env->solutionStatistics.numberOfProblemsLP > 0)
    {
        report << "  - LP problems                                  " << env->solutionStatistics.numberOfProblemsLP << "\r\n";
    }

    if (env->solutionStatistics.numberOfProblemsQP > 0)
    {
        report << "  - QP problems                                  " << env->solutionStatistics.numberOfProblemsQP << "\r\n";
    }

    if (env->solutionStatistics.numberOfProblemsQCQP > 0)
    {
        report << "  - QCQP problems                                " << env->solutionStatistics.numberOfProblemsQCQP << "\r\n";
    }

    if (env->solutionStatistics.numberOfProblemsOptimalMILP > 0)
    {
        report << "  - MILP problems, optimal                       " << env->solutionStatistics.numberOfProblemsOptimalMILP << "\r\n";
    }

    if (env->solutionStatistics.numberOfProblemsFeasibleMILP > 0)
    {
        report << "  - MILP problems, feasible                      " << env->solutionStatistics.numberOfProblemsFeasibleMILP << "\r\n";
    }

    if (env->solutionStatistics.numberOfProblemsOptimalMIQP > 0)
    {
        report << "  - MIQP problems, optimal                       " << env->solutionStatistics.numberOfProblemsOptimalMIQP << "\r\n";
    }

    if (env->solutionStatistics.numberOfProblemsFeasibleMIQP > 0)
    {
        report << "  - MIQP problems, feasible                      " << env->solutionStatistics.numberOfProblemsFeasibleMIQP << "\r\n";
    }

    report << "\r\n";

    if (env->solutionStatistics.numberOfExploredNodes > 0)
    {
        report << " Number of explored nodes:                       ";
        report << env->solutionStatistics.numberOfExploredNodes << "\r\n";

        report << "\r\n";
    }

    if (env->solutionStatistics.numberOfProblemsNLPInteriorPointSearch > 0 ||
        env->solutionStatistics.numberOfProblemsMinimaxLP > 0)
    {
        report << " Problems solved during interior point search: \r\n";

        if (env->solutionStatistics.numberOfProblemsNLPInteriorPointSearch > 0)
        {
            report << " - NLP problems:                                 ";
            report << env->solutionStatistics.numberOfProblemsNLPInteriorPointSearch << "\r\n";
        }

        if (env->solutionStatistics.numberOfProblemsMinimaxLP > 0)
        {
            report << " - LP problems:                                  ";
            report << env->solutionStatistics.numberOfProblemsMinimaxLP << "\r\n";
        }
    }

    report << "\r\n";

    if (env->solutionStatistics.numberOfProblemsFixedNLP > 0)
    {
        report << " Fixed primal NLP problems solved:               ";
        report << env->solutionStatistics.numberOfProblemsFixedNLP << "\r\n";
    }

    report << "\r\n";

    for (auto T : env->process->timers)
    {
        T.stop();
        auto elapsed = T.elapsed();

        if (elapsed > 0)
        {
            report << boost::format(" %1%: %|49t|%2%") % T.description % elapsed << "\r\n";
        }
    }

    env->output->outputSummary(report.str());
}

void Report::outputInteriorPointPreReport()
{
    std::stringstream report;

    report << "\r\n";
    report << "╶ Interior point search ──────────────────────────────────────────────────────────────────────────────────────────────╴\r\n";
    report << "\r\n";

    report << " Strategy selected:          ";

    switch (static_cast<ES_InteriorPointStrategy>(env->settings->getIntSetting("ESH.InteriorPoint.Solver", "Dual")))
    {
    case (ES_InteriorPointStrategy::CuttingPlaneMiniMax):
        report << "cutting plane minimax";
        break;
    case (ES_InteriorPointStrategy::IpoptMinimax):
        report << "Ipopt minimax";
        break;
    case (ES_InteriorPointStrategy::IpoptRelaxed):
        report << "Ipopt relaxed";
        break;
    case (ES_InteriorPointStrategy::IpoptMinimaxAndRelaxed):
        report << "Ipopt minimax and Ipopt relaxed";
        break;
    default:
        report << "none";
        break;
    }

    env->output->outputSummary(report.str());
}
} // namespace SHOT