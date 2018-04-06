/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "Output.h"

Output::Output()
{
    osOutput = new OSOutput();

    // Adds a file output
    osOutput->AddChannel("shotlogfile");
    osOutput->AddChannel("stdout");
}

Output::~Output()
{
    delete osOutput;
    osOutput = NULL;
}

void Output::outputAlways(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_always, message);
}

void Output::outputError(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error, message);
}

void Output::outputError(std::string message, std::string errormessage)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error, message);
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error, " \"" + errormessage + "\"");
}

void Output::outputSummary(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_summary, message);
}

void Output::outputWarning(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_warning, message);
}

void Output::outputInfo(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info, message);
}

void Output::outputDebug(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_debug, message);
}

void Output::outputTrace(std::string message)
{
#ifndef NDEBUG
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_trace, message);
#endif
}

void Output::outputDetailedTrace(std::string message)
{
#ifndef NDEBUG
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_detailed_trace, message);
#endif
}

void Output::setLogLevels()
{
    // Sets the correct log levels
    osOutput->SetPrintLevel("stdout",
                            (ENUM_OUTPUT_LEVEL)(Settings::getInstance().getIntSetting("Console.LogLevel", "Output") + 1));
    osOutput->SetPrintLevel("shotlogfile",
                            (ENUM_OUTPUT_LEVEL)(Settings::getInstance().getIntSetting("File.LogLevel", "Output") + 1));
}

void Output::outputIterationDetail(int iterationNumber,
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
                                   double maxConstraintError)
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

        outputSummary(tmpLine.str());
    }
    catch (...)
    {
        outputError("ERROR, cannot write iteration solution report!");
    }
}

void Output::outputIterationDetailMinimax(int iterationNumber,
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

        outputSummary(tmpLine.str());
    }
    catch (...)
    {
        outputError("ERROR, cannot write iteration solution report!");
    }
}

void Output::outputIterationDetailHeader()
{
    std::stringstream header;

    header << "\r\n";
    header << "                                                                                     \n";

    header << "    Iteration     │  Time  │  Dual cuts  │     Objective value     │   Objective gap   │     Current solution\r\n";
    header << "     #: type      │  tot.  │   + | tot.  │       dual | primal     │    abs. | rel.    │    obj.fn. | max.err.\r\n";
    header << "╶─────────────────┴────────┴─────────────┴─────────────────────────┴───────────────────┴───────────────────────────╴\r\n";
    header << "\r\n";

    outputSummary(header.str());
}

void Output::outputIterationDetailHeaderMinimax()
{
    std::stringstream header;
    header << "                                                                                     \n";

    header << "    Iteration     │  Time  │    Cuts     │     Objective value     │  Objective diff.   \r\n";
    header << "     #: type      │  tot.  │   + | tot.  │    problem | line srch  │    abs. | rel.    \r\n";
    header << "╶─────────────────┴────────┴─────────────┴─────────────────────────┴──────────────────╴\r\n";

    outputSummary(header.str());
}

void Output::outputSolverHeader()
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

    outputSummary(header.str());
}

void Output::outputOptionsReport()
{
    std::stringstream report;

    report << "\r\n";
    report << "╶ Options ────────────────────────────────────────────────────────────────────────────────────────────────────────────╴\r\n";
    report << "\r\n";

    std::string optionsFile = Settings::getInstance().getStringSetting("OptionsFile", "Input");

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

    report << " Nondefault options used:\r\n";
    report << "\r\n";
    report << Settings::getInstance().getUpdatedSettingsAsString();

    report << "\r\n";

    std::string cutAlgorithm, dualSolver;
    bool useSingleTree = (static_cast<ES_TreeStrategy>(Settings::getInstance().getIntSetting("TreeStrategy", "Dual")) == ES_TreeStrategy::SingleTree);

    if (static_cast<ES_HyperplaneCutStrategy>(Settings::getInstance().getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
    {
        cutAlgorithm = "ESH";
    }
    else
    {
        cutAlgorithm = "ECP";
    }

    auto solver = static_cast<ES_MIPSolver>(Settings::getInstance().getIntSetting("MIP.Solver", "Dual"));

    if (useSingleTree)
    {
#ifdef HAS_CPLEX
        if (solver == ES_MIPSolver::Cplex)
        {
#ifdef HAS_CPLEX_NEW_CALLBACK
            if (Settings::getInstance().getBoolSetting("Cplex.UseNewCallbackType", "Subsolver"))
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

    switch (static_cast<E_SolutionStrategy>(ProcessInfo::getInstance().usedSolutionStrategy))
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

    switch (static_cast<ES_PrimalNLPSolver>(ProcessInfo::getInstance().usedPrimalNLPSolver))
    {
    case (ES_PrimalNLPSolver::None):
        report << "none";
        break;
    case (ES_PrimalNLPSolver::CuttingPlane):
        report << "cutting plane";
        break;
    case (ES_PrimalNLPSolver::GAMS):
        report << "GAMS (";
        report << Settings::getInstance().getStringSetting("GAMS.NLP.Solver", "Subsolver");
        report << ")\r\n";
        break;
    case (ES_PrimalNLPSolver::Ipopt):
        report << "Ipopt (";

        switch (static_cast<ES_IpoptSolver>(Settings::getInstance().getIntSetting("Ipopt.LinearSolver", "Subsolver")))
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

    if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
    {
        report << " Debug directory:            ";
        report << Settings::getInstance().getStringSetting("Debug.Path", "Output");
        report << "\r\n";
    }

    outputSummary(report.str());
}

void Output::outputProblemInstanceReport()
{
    std::stringstream report;

    auto problemStats = ProcessInfo::getInstance().problemStats;

    report << "\r\n";
    report << "╶ Problem instance ───────────────────────────────────────────────────────────────────────────────────────────────────╴\r\n";
    report << "\r\n";

    std::string problemFile = Settings::getInstance().getStringSetting("ProblemFile", "Input");

    report << " Problem read from file:     " << problemFile;
    report << "\r\n";

    report << " Objective function type:    ";

    switch (static_cast<E_ObjectiveFunctionType>(problemStats.objectiveFunctionType))
    {
    case (E_ObjectiveFunctionType::Linear):
        report << "linear";
        break;

    case (E_ObjectiveFunctionType::Nonlinear):
        report << "nonlinear";
        break;

    case (E_ObjectiveFunctionType::Quadratic):
        report << "quadratic";
        break;

    case (E_ObjectiveFunctionType::QuadraticConsideredAsNonlinear):
        report << "quadratic but considered as nonlinear";
        break;

    default:
        report << "unknown";
        break;
    }

    report << "\r\n";
    report << "\r\n";

    report << " Number of constraints:      " << problemStats.numberOfConstraints << "\r\n";

    if (problemStats.numberOfLinearConstraints > 0)
        report << "  - linear:                  " << problemStats.numberOfLinearConstraints << "\r\n";
    if (problemStats.numberOfNonlinearConstraints > 0)
        report << "  - nonlinear:               " << problemStats.numberOfNonlinearConstraints << "\r\n";
    if (problemStats.numberOfQuadraticConstraints > 0)
    {
        report << "  - quadratic:               " << problemStats.numberOfQuadraticConstraints;

        if (problemStats.quadraticTermsReformulatedAsNonlinear)
            report << " (considered as nonlinear)\r\n";
        else
            report << "\r\n";
    }

    report << "\r\n";

    report << " Number of variables:        " << problemStats.numberOfVariables << "\r\n";
    if (problemStats.numberOfContinousVariables > 0)
        report << "  - continous:               " << problemStats.numberOfContinousVariables << "\r\n";
    if (problemStats.numberOfBinaryVariables > 0)
        report << "  - binary:                  " << problemStats.numberOfBinaryVariables << "\r\n";
    if (problemStats.numberOfIntegerVariables > 0)
        report << "  - integer:                 " << problemStats.numberOfIntegerVariables << "\r\n";
    if (problemStats.numberOfSemicontinuousVariables > 0)
        report << "  - semicontinuous:          " << problemStats.numberOfSemicontinuousVariables << "\r\n";

    outputSummary(report.str());
}

void Output::outputSolutionReport()
{
    std::stringstream report;
    auto problemStats = ProcessInfo::getInstance().problemStats;

    report << "\r\n";
    report << "╶ Solution report ────────────────────────────────────────────────────────────────────────────────────────────────────╴\r\n";
    report << "\r\n";

    auto terminationReason = ProcessInfo::getInstance().terminationReason;

    bool primalSolutionFound = (ProcessInfo::getInstance().primalSolutions.size() > 0);

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

    report << " Objective bound [dual, primal]:                ";
    report << "[" << UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getDualBound(), "%g") << ", ";
    report << UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getPrimalBound(), "%g") << "]\r\n";
    report << " Objective gap absolute / relative:             ";
    report << "[" << UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getAbsoluteObjectiveGap(), "%g") << ", ";
    report << UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getRelativeObjectiveGap(), "%g") << "]\r\n";
    report << "\r\n";

    std::stringstream fulfilled;
    std::stringstream unfulfilled;

    if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
    {
        fulfilled << "  - absolute objective gap tolerance 		";
        fulfilled << ProcessInfo::getInstance().getAbsoluteObjectiveGap() << " <= ";
        fulfilled << Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination") << "\r\n";
    }
    else
    {
        unfulfilled << "  - absolute objective gap tolerance 		";
        unfulfilled << ProcessInfo::getInstance().getAbsoluteObjectiveGap() << " > ";
        unfulfilled << Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination") << "\r\n";
    }

    if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
    {
        fulfilled << "  - relative objective gap tolerance 		";
        fulfilled << ProcessInfo::getInstance().getRelativeObjectiveGap() << " <= ";
        fulfilled << Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination") << "\r\n";
    }
    else
    {
        unfulfilled << "  - relative objective gap tolerance 		";
        unfulfilled << ProcessInfo::getInstance().getRelativeObjectiveGap() << " > ";
        unfulfilled << Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination") << "\r\n";
    }

    if (ProcessInfo::getInstance().getCurrentIteration()->maxDeviation <= Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"))
    {
        fulfilled << "  - maximal constraint tolerance 	        ";
        fulfilled << ProcessInfo::getInstance().getCurrentIteration()->maxDeviation << " <= ";
        fulfilled << Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination") << "\r\n";
    }
    else
    {
        unfulfilled << "  - maximal constraint tolerance 	        ";
        unfulfilled << ProcessInfo::getInstance().getCurrentIteration()->maxDeviation << " > ";
        unfulfilled << Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination") << "\r\n";
    }

    int iterLim = Settings::getInstance().getIntSetting("Relaxation.IterationLimit", "Dual") + Settings::getInstance().getIntSetting("IterationLimit", "Termination");

    if (ProcessInfo::getInstance().getCurrentIteration()->iterationNumber > iterLim)
    {
        fulfilled << "  - iteration limit                           ";
        fulfilled << ProcessInfo::getInstance().getCurrentIteration()->iterationNumber << " > ";
        fulfilled << iterLim << "\r\n";
    }
    else
    {
        unfulfilled << "  - iteration limit                             ";
        unfulfilled << ProcessInfo::getInstance().getCurrentIteration()->iterationNumber << " <= ";
        unfulfilled << iterLim << "\r\n";
    }

    if (ProcessInfo::getInstance().getElapsedTime("Total") > Settings::getInstance().getDoubleSetting("TimeLimit", "Termination"))
    {
        fulfilled << "  - solution time limit (s)                   ";
        fulfilled << ProcessInfo::getInstance().getElapsedTime("Total") << " > ";
        fulfilled << Settings::getInstance().getDoubleSetting("TimeLimit", "Termination") << "\r\n";
    }
    else
    {
        unfulfilled << "  - solution time limit (s)                     ";
        unfulfilled << ProcessInfo::getInstance().getElapsedTime("Total") << " <= ";
        unfulfilled << Settings::getInstance().getDoubleSetting("TimeLimit", "Termination") << "\r\n";
    }

    report << " Fulfilled termination criteria: \r\n";
    report << fulfilled.str();
    report << "\r\n";

    report << " Unfulfilled termination criteria: \r\n";
    report << unfulfilled.str();
    report << "\r\n";

    report << " Dual problems solved in main step:             ";
    report << ProcessInfo::getInstance().solutionStatistics.getNumberOfTotalDualProblems() << "\r\n";

    if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsLP > 0)
    {
        report << "  - LP problems                                 " << ProcessInfo::getInstance().solutionStatistics.numberOfProblemsLP << "\r\n";
    }

    if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsQP > 0)
    {
        report << "  - QP problems                                 " << ProcessInfo::getInstance().solutionStatistics.numberOfProblemsQP << "\r\n";
    }

    if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsQCQP > 0)
    {
        report << "  - QCQP problems                               " << ProcessInfo::getInstance().solutionStatistics.numberOfProblemsQCQP << "\r\n";
    }

    if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsOptimalMILP > 0)
    {
        report << "  - MILP problems, optimal                      " << ProcessInfo::getInstance().solutionStatistics.numberOfProblemsOptimalMILP << "\r\n";
    }

    if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsFeasibleMILP > 0)
    {
        report << "  - MILP problems, feasible                     " << ProcessInfo::getInstance().solutionStatistics.numberOfProblemsFeasibleMILP << "\r\n";
    }

    if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsOptimalMIQP > 0)
    {
        report << "  - MIQP problems, optimal                      " << ProcessInfo::getInstance().solutionStatistics.numberOfProblemsOptimalMIQP << "\r\n";
    }

    if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsFeasibleMIQP > 0)
    {
        report << "  - MIQP problems, feasible                     " << ProcessInfo::getInstance().solutionStatistics.numberOfProblemsFeasibleMIQP << "\r\n";
    }

    report << "\r\n";

    if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsNLPInteriorPointSearch > 0 ||
        ProcessInfo::getInstance().solutionStatistics.numberOfProblemsMinimaxLP > 0)
    {
        report << " Problems solved during interior point search: \r\n";

        if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsNLPInteriorPointSearch > 0)
        {
            report << " - NLP problems:                                ";
            report << ProcessInfo::getInstance().solutionStatistics.numberOfProblemsNLPInteriorPointSearch << "\r\n";
        }

        if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsMinimaxLP > 0)
        {
            report << " - LP problems:                                 ";
            report << ProcessInfo::getInstance().solutionStatistics.numberOfProblemsMinimaxLP << "\r\n";
        }
    }

    report << "\r\n";

    if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsFixedNLP > 0)
    {
        report << " Fixed primal NLP problems solved:              ";
        report << ProcessInfo::getInstance().solutionStatistics.numberOfProblemsFixedNLP << "\r\n";
    }

    report << "\r\n";

    for (auto T : ProcessInfo::getInstance().timers)
    {
        T.stop();
        auto elapsed = T.elapsed();

        if (elapsed > 0)
        {
            report << boost::format(" %1%: %|48t|%2%") % T.description % elapsed << "\r\n";
        }
    }

    outputSummary(report.str());
}

void Output::outputInteriorPointPreReport()
{
    std::stringstream report;

    report << "\r\n";
    report << " ─ Interior point search ────────────────────────────────────────────────────────────────────────\r\n";
    report << "\r\n";

    report << " Strategy selected:          ";

    switch (static_cast<ES_InteriorPointStrategy>(Settings::getInstance().getIntSetting("ESH.InteriorPoint.Solver", "Dual")))
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

    outputSummary(report.str());
}
