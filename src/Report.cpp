/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Report.h"

#include "DualSolver.h"
#include "Iteration.h"
#include "MIPSolver/IMIPSolver.h"
#include "Output.h"
#include "PrimalSolver.h"
#include "Results.h"
#include "Settings.h"
#include "TaskHandler.h"
#include "Timing.h"
#include "Utilities.h"

#include "Model/Problem.h"

namespace SHOT
{

Report::Report(EnvironmentPtr envPtr) : env(envPtr) {}

Report::~Report() = default;

void Report::outputIterationDetail(int iterationNumber, std::string iterationDesc, double totalTime, int dualCutsAdded,
    int dualCutsTotal, double dualObjectiveValue, double primalObjectiveValue, double absoluteObjectiveGap,
    double relativeObjectiveGap, double currentObjectiveValue, int maxConstraintIndex, double maxConstraintError,
    E_IterationLineType lineType, bool forcePrint)
{
    try
    {
        bool printLine = forcePrint;

        if(env->results->getNumberOfIterations() == 1)
        {
            printLine = true;
        }

        if(dualObjectiveValue != lastDualObjectiveValue)
        {
            lastDualObjectiveValue = dualObjectiveValue;
            printLine = true;
        }

        if(primalObjectiveValue != lastPrimalObjectiveValue)
        {
            lastPrimalObjectiveValue = primalObjectiveValue;
            printLine = true;
        }

        if(iterationsWithoutPrintoutCounter > 100
            || env->timing->getElapsedTime("Total") - lastIterationOutputTimeStamp > 2)
        {
            printLine = true;
        }

        switch(
            static_cast<ES_IterationOutputDetail>(env->settings->getSetting<int>("Console.Iteration.Detail", "Output")))
        {
        case ES_IterationOutputDetail::Full:
            printLine = true;
            break;

        case ES_IterationOutputDetail::ObjectiveGapUpdatesAndNLPCalls:
            if(lineType == E_IterationLineType::PrimalNLP)
                printLine = true;
            break;

        default:
            break;
        }

        if(!printLine)
        {
            iterationsWithoutPrintoutCounter++;
            return;
        }

        if(!firstIterationHeaderPrinted)
        {
            this->outputIterationDetailHeader();
        }

        if(iterationPrintoutsSinceLastHeader > 75)
        {
            this->outputIterationDetailHeader();
        }

        iterationsWithoutPrintoutCounter = 0;
        iterationPrintoutsSinceLastHeader++;
        lastIterationOutputTimeStamp = env->timing->getElapsedTime("Total");

        std::string combDualCuts = "";

        if(lineType == E_IterationLineType::DualRepair)
        {
            combDualCuts = fmt::format("Repairs: {:<4d}", dualCutsAdded);
        }
        else if(lineType == E_IterationLineType::DualReductionCut)
        {
            combDualCuts = fmt::format("Obj.cut: {:<4g}", primalObjectiveValue);
        }
        else if(dualCutsAdded > 0)
        {
            combDualCuts = fmt::format("{:>4d} | {:<6d}", dualCutsAdded, dualCutsTotal);
        }

        std::string combObjectiveValue;
        if(env->problem->objectiveFunction->properties.isMinimize)
        {
            combObjectiveValue = fmt::format("{:>12s}{}| {:<12s}",
                Utilities::toStringFormat(dualObjectiveValue, "{:g}"), env->results->solutionIsGlobal ? " " : "*",
                Utilities::toStringFormat(primalObjectiveValue, "{:g}"));
        }
        else
        {
            combObjectiveValue
                = fmt::format("{:>12s} |{}{:<12s}", Utilities::toStringFormat(primalObjectiveValue, "{:g}"),
                    env->results->solutionIsGlobal ? " " : "*", Utilities::toStringFormat(dualObjectiveValue, "{:g}"));
        }

        std::string combObjectiveGap
            = fmt::format("{:>8s} | {:<8s}", Utilities::toStringFormat(absoluteObjectiveGap, "{:.1e}"),
                Utilities::toStringFormat(relativeObjectiveGap, "{:.1e}"));

        std::string combCurrSol;

        if(std::isnan(currentObjectiveValue))
        {
            combCurrSol = fmt::format("{:>12s} | {}", "", "inf.");
        }
        else if(env->reformulatedProblem->properties.numberOfNonlinearConstraints == 0)
        {
            combCurrSol = fmt::format("{:>12g} | {}", currentObjectiveValue, 0.0);
        }
        else
        {
            combCurrSol
                = fmt::format("{:>12g} | {}: {:.2e}", currentObjectiveValue, maxConstraintIndex, maxConstraintError);
        }

        if(lineType == E_IterationLineType::DualRepair || lineType == E_IterationLineType::DualReductionCut)
        {
            auto tmpLine = fmt::format("{:>6d}: {:<10s}{:^10.2f}{:^13s}{:>27s}{:>19s}{:<32s}", iterationNumber,
                iterationDesc, totalTime, combDualCuts, "", "", "");

            env->output->outputDebug("");
            env->output->outputInfo(tmpLine);
            env->output->outputDebug("");
        }
        else
        {
            auto tmpLine = fmt::format("{:>6d}: {:<10s}{:^10.2f}{:>13s}{:>27s}{:>19s}{:<32s}", iterationNumber,
                iterationDesc, totalTime, combDualCuts, combObjectiveValue, combObjectiveGap, combCurrSol);

            env->output->outputDebug("");
            env->output->outputInfo(tmpLine);
            env->output->outputDebug("");
        }

        std::stringstream nodes;
        nodes << "        Explored nodes: ";

        if(env->results->getCurrentIteration()->numberOfExploredNodes > 0)
        {
            nodes << " +" << env->results->getCurrentIteration()->numberOfExploredNodes << " = ";
        }

        nodes << env->solutionStatistics.numberOfExploredNodes << ".";

        if(env->results->getCurrentIteration()->numberOfOpenNodes > 0)
        {
            nodes << " Open nodes: " << env->results->getCurrentIteration()->numberOfOpenNodes << ".";
        }

        env->output->outputDebug(nodes.str());
    }
    catch(...)
    {
        env->output->outputError("        Cannot write iteration solution report!");
    }
}

void Report::outputIterationDetailMinimax(int iterationNumber, std::string iterationDesc, double totalTime,
    int dualCutsAdded, int dualCutsTotal, double dualObjectiveValue, double primalObjectiveValue,
    double absoluteObjectiveGap, double relativeObjectiveGap)
{
    try
    {
        std::string combDualCuts = "";

        if(dualCutsAdded > 0)
        {
            combDualCuts = fmt::format("{:>4d} | {:<6d}", dualCutsAdded, dualCutsTotal);
        }

        if(dualObjectiveValue != lastDualObjectiveValue)
        {
            lastDualObjectiveValue = dualObjectiveValue;
        }

        if(primalObjectiveValue != lastPrimalObjectiveValue)
        {
            lastPrimalObjectiveValue = primalObjectiveValue;
        }

        std::string combObjectiveValue
            = fmt::format("{:>12s} | {:<12s}", Utilities::toStringFormat(dualObjectiveValue, "{:g}"),
                Utilities::toStringFormat(primalObjectiveValue, "{:g}"));

        if(absoluteObjectiveGap != lastAbsoluteObjectiveGap)
        {
            lastAbsoluteObjectiveGap = absoluteObjectiveGap;
        }

        if(relativeObjectiveGap != lastRelativeObjectiveGap)
        {
            lastRelativeObjectiveGap = relativeObjectiveGap;
        }

        std::string combObjectiveGap
            = fmt::format("{:>8s} | {:<8s}", Utilities::toStringFormat(absoluteObjectiveGap, "{:.1e}"),
                Utilities::toStringFormat(relativeObjectiveGap, "{:.1e}"));

        auto tmpLine = fmt::format("{:6d}: {:<10s}{:^10.2f}{:13s}{:27s}{:19s}", iterationNumber, iterationDesc,
            totalTime, combDualCuts, combObjectiveValue, combObjectiveGap);

        env->output->outputInfo(tmpLine);
    }
    catch(...)
    {
        env->output->outputError("Cannot write iteration solution report!");
    }
}

void Report::outputIterationDetailHeader()
{
    firstIterationHeaderPrinted = true;

    std::stringstream header;

    header << "\n";
    header << "                                                                                     \n";

#ifdef SIMPLE_OUTPUT_CHARS
    header << "    Iteration     |  Time  |  Dual cuts  |     Objective value     |   Objective gap   |     Current "
              "solution\n";

    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        header
            << "     #: type      |  tot.  |   + | tot.  |       dual | primal     |    abs. | rel.    |    obj.fn. | "
               "max.err.\n";
    }
    else
    {
        header
            << "     #: type      |  tot.  |   + | tot.  |     primal | dual       |    abs. | rel.    |    obj.fn. | "
               "max.err.\n";
    }

    header << "----------------------------------------------------------------------------------------------------"
              "----------------";
#else
    header << "    Iteration     │  Time  │  Dual cuts  │     Objective value     │   Objective gap   │     Current "
              "solution\n";

    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        header
            << "     #: type      │  tot.  │   + | tot.  │       dual | primal     │    abs. | rel.    │    obj.fn. | "
               "max.err.\n";
    }
    else
    {
        header
            << "     #: type      │  tot.  │   + | tot.  │     primal | dual       │    abs. | rel.    │    obj.fn. | "
               "max.err.\n";
    }

    header << "╶─────────────────┴────────┴─────────────┴─────────────────────────┴───────────────────┴────────────"
              "───────────────╴";
#endif

    header << "\n";

    env->output->outputInfo(header.str());
    iterationPrintoutsSinceLastHeader = 0;
}

void Report::outputIterationDetailHeaderMinimax()
{
    std::stringstream header;
    header << "                                                                                     \n";

#ifdef SIMPLE_OUTPUT_CHARS
    header << "    Iteration      |  Time  |    Cuts     |     Objective value     |  Objective diff.   \n";
    header << "     #: type       |  tot.  |   + | tot.  |    problem | line srch  |    abs. | rel.    \n";
    header << "---------------------------------------------------------------------------------------\n";
#else
    header << "    Iteration     │  Time  │    Cuts     │     Objective value     │  Objective diff.   \n";
    header << "     #: type      │  tot.  │   + | tot.  │    problem | line srch  │    abs. | rel.    \n";
    header << "╶─────────────────┴────────┴─────────────┴─────────────────────────┴──────────────────╴\n";
#endif

    env->output->outputInfo(header.str());
}

void Report::outputSolverHeader()
{
    std::stringstream header;

#ifdef SIMPLE_OUTPUT_CHARS
    header << "\n";
    header << "- Supporting Hyperplane Optimization Toolkit (SHOT) ";
    header << "-------------------------------------------------------------------\n";
#else
    header << "\n";
    header << "╶ Supporting Hyperplane Optimization Toolkit (SHOT) ";
    header << "──────────────────────────────────────────────────────────────────╴\n";
#endif

    header << "\n";

    header << " Andreas Lundell and Jan Kronqvist, Åbo Akademi University, Finland.\n";
    header << " See documentation for full list of contributors and utilized software libraries.\n";

    header << "\n";
    header << " Version: ";
    header << SHOT_VERSION_MAJOR;
    header << ".";
    header << SHOT_VERSION_MINOR;

    if(SHOT_VERSION_PATCH != 0)
    {
        header << ".";
        header << SHOT_VERSION_PATCH;
    }

    header << ". ";

    if(*SHOT_GITHASH != '\0')
    {
        header << "Git hash: ";
        header << SHOT_GITHASH;
        header << ". ";
    }

    header << "Released ";
    header << __DATE__;
    header << ". ";

    header << "\n";
    header << "\n";
    header << " For more information visit https://shotsolver.dev\n";

    env->output->outputInfo(header.str());
}

void Report::outputOptionsReport()
{
    std::stringstream report;

    report << "\n";

#ifdef SIMPLE_OUTPUT_CHARS
    report << "- Options ";
    report << "--------------------------------------------------------------------------------------------------------"
              "-----\n";
#else
    report << "╶ Options ";
    report << "────────────────────────────────────────────────────────────────────────────────────────────────────────"
              "────╴\n";
#endif

    report << "\n";

    auto optionsFile = env->settings->getSetting<std::string>("OptionsFile", "Input");

    if(optionsFile == "")
    {
        report << " No options file specified.\n";
    }
    else
    {
        report << " Options read from file:     ";
        report << optionsFile;
        report << "\n";
    }

    report << "\n";

    auto nonDefaultSettings = env->settings->getChangedSettings();

    if(nonDefaultSettings.size() > 0)
    {
        report << " Nondefault options used:\n";
        report << "\n";

        for(auto& S : nonDefaultSettings)
        {
            report << "  - " << S << "\n";
        }

        report << "\n";
    }

    std::string cutAlgorithm, dualSolver;
    bool useSingleTree = (static_cast<ES_TreeStrategy>(env->settings->getSetting<int>("TreeStrategy", "Dual"))
        == ES_TreeStrategy::SingleTree);

    if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::ESH)
    {
        cutAlgorithm = "ESH";
    }
    else
    {
        cutAlgorithm = "ECP";
    }

    auto solver = static_cast<ES_MIPSolver>(env->settings->getSetting<int>("MIP.Solver", "Dual"));

    if(useSingleTree)
    {
#ifdef HAS_CPLEX
        if(solver == ES_MIPSolver::Cplex)
        {
            dualSolver = "CPLEX";
        }
#endif

#ifdef HAS_GUROBI
        if(solver == ES_MIPSolver::Gurobi)
        {
            dualSolver = "Gurobi";
        }
#endif

#ifdef HAS_CBC
        if(solver == ES_MIPSolver::Cbc)
        {
            dualSolver = "Cbc";
        }
#endif
    }
    else
    {

#ifdef HAS_CPLEX
        if(solver == ES_MIPSolver::Cplex)
        {
            dualSolver = "CPLEX";
        }
#endif

#ifdef HAS_GUROBI
        if(solver == ES_MIPSolver::Gurobi)
        {
            dualSolver = "Gurobi";
        }
#endif

#ifdef HAS_CBC
        if(solver == ES_MIPSolver::Cbc)
        {
            dualSolver = "Cbc";
        }
#endif
    }

    switch(static_cast<E_SolutionStrategy>(env->results->usedSolutionStrategy))
    {
    case(E_SolutionStrategy::SingleTree):
        report << " Dual strategy:              Single-tree\n";
        report << "  - cut algorithm:           " << cutAlgorithm << "\n";
        break;
    case(E_SolutionStrategy::MultiTree):
        report << " Dual strategy:              Multi-tree\n";
        report << "  - cut algorithm:           " << cutAlgorithm << "\n";
        break;
    case(E_SolutionStrategy::NLP):
        report << " Dual strategy:              NLP version\n";
        report << "  - cut algorithm:           " << cutAlgorithm << "\n";
        break;
    case(E_SolutionStrategy::MIQP):
        report << " Dual strategy:              MIQP version\n";
        break;
    case(E_SolutionStrategy::MIQCQP):
        report << " Dual strategy:              MIQCQP version\n";
        break;
    default:
        break;
    }

    report << "  - solver:                  " << dualSolver << " " << env->dualSolver->MIPSolver->getSolverVersion()
           << "\n";

    report << "\n";

    report << " Primal NLP solver:          ";

    if(static_cast<ES_PrimalNLPSolver>(env->results->usedPrimalNLPSolver) == ES_PrimalNLPSolver::None)
        report << "none";
    else
        report << env->results->usedPrimalNLPSolverDescription;

    report << "\n";

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        report << " Debug directory:            ";
        report << env->settings->getSetting<std::string>("Debug.Path", "Output");
        report << "\n";
    }

    env->output->outputInfo(report.str());
}

void Report::outputModelingSystemHeader(ES_SourceFormat source, std::string filename)
{
    std::stringstream report;

#ifdef SIMPLE_OUTPUT_CHARS
    report << "- Modeling system ";
    report << "-----------------------------------------------------------------------------------------------------"
              "\n";
#else
    report << "╶ Modeling system ";
    report << "────────────────────────────────────────────────────────────────────────────────────────────────────╴"
              "\n";
#endif

    report << "\n";

    switch(source)
    {
    case(ES_SourceFormat::GAMS):
        report << " Modeling system:            GAMS\n";
        break;
    case(ES_SourceFormat::OSiL):
        report << " Modeling system:            OSiL\n";
        break;

    case(ES_SourceFormat::NL):
        report << " Modeling system:            AMPL\n";
        break;

    default:
        break;
    }

    if(filename != "")
        report << " Problem read from file:     " << filename << "\n";

    env->output->outputInfo(report.str());
}

void Report::outputModelingSystemReport()
{
    std::stringstream report;
    env->output->outputInfo(report.str());
}

void Report::outputProblemInstanceReport()
{
    std::stringstream report;

    bool isReformulated = (env->problem == env->reformulatedProblem) ? false : true;

#ifdef SIMPLE_OUTPUT_CHARS
    report << "\n- Problem instance ";
    report << "----------------------------------------------------------------------------------------------------"
              "\n";
#else
    report << "\n╶ Problem instance ";
    report << "───────────────────────────────────────────────────────────────────────────────────────────────────╴"
              "\n";
#endif

    report << "\n";

    if(isReformulated)
    {
        report << fmt::format(" {:35s}{:21s}{:s}", "", "Original", "Reformulated") << "\n";
    }
    else
    {
        report << fmt::format(" {:35s}{:21s}{:s}", "", "Original", "") << "\n";
    }

    report << "\n";

    std::string problemClassificationOrig;
    std::string problemClassificationRef = "";

    if(env->problem->properties.isLPProblem)
        problemClassificationOrig = "LP";
    else if(env->problem->properties.isMILPProblem)
        problemClassificationOrig = "MILP";
    else if(env->problem->properties.isQPProblem)
        problemClassificationOrig = "QP";
    else if(env->problem->properties.isQCQPProblem)
        problemClassificationOrig = "QCQP";
    else if(env->problem->properties.isMIQPProblem)
        problemClassificationOrig = "MIQP";
    else if(env->problem->properties.isMIQCQPProblem)
        problemClassificationOrig = "MIQCQP";
    else if(env->problem->properties.isNLPProblem)
        problemClassificationOrig = "NLP";
    else if(env->problem->properties.isMINLPProblem)
        problemClassificationOrig = "MINLP";

    if(env->problem->properties.convexity == E_ProblemConvexity::Convex)
        problemClassificationOrig += ", convex";
    else if(env->problem->properties.convexity == E_ProblemConvexity::Nonconvex)
        problemClassificationOrig += ", nonconvex";

    if(isReformulated)
    {
        if(env->reformulatedProblem->properties.isLPProblem)
            problemClassificationRef = "LP";
        else if(env->reformulatedProblem->properties.isMILPProblem)
            problemClassificationRef = "MILP";
        else if(env->reformulatedProblem->properties.isQPProblem)
            problemClassificationRef = "QP";
        else if(env->reformulatedProblem->properties.isQCQPProblem)
            problemClassificationRef = "QCQP";
        else if(env->reformulatedProblem->properties.isMIQPProblem)
            problemClassificationRef = "MIQP";
        else if(env->reformulatedProblem->properties.isMIQCQPProblem)
            problemClassificationRef = "MIQCQP";
        else if(env->reformulatedProblem->properties.isNLPProblem)
            problemClassificationRef = "NLP";
        else if(env->reformulatedProblem->properties.isMINLPProblem)
            problemClassificationRef = "MINLP";

        if(env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Convex)
            problemClassificationRef += ", convex";
        else if(env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Nonconvex)
            problemClassificationRef += ", nonconvex";
    }

    report << fmt::format(
                  " {:35s}{:21s}{:s}", "Problem classification:", problemClassificationOrig, problemClassificationRef)
           << "\n";

    std::string objectiveClassificationOrig;
    std::string objectiveClassificationRef = "";
    std::string objectiveDirectionOrig
        = env->problem->objectiveFunction->properties.isMinimize ? "minimize" : "maximize";
    std::string objectiveDirectionRef = "";

    switch(static_cast<E_ObjectiveFunctionClassification>(env->problem->objectiveFunction->properties.classification))
    {
    case(E_ObjectiveFunctionClassification::Linear):
        objectiveClassificationOrig = "linear";
        break;

    case(E_ObjectiveFunctionClassification::Quadratic):
        objectiveClassificationOrig = "quadratic";
        break;

    case(E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear):
        objectiveClassificationOrig = "quadratic but considered as nonlinear";
        break;

    case(E_ObjectiveFunctionClassification::Signomial):
        objectiveClassificationOrig = "signomial";
        break;

    case(E_ObjectiveFunctionClassification::Nonlinear):
        objectiveClassificationOrig = "nonlinear";
        break;

    case(E_ObjectiveFunctionClassification::GeneralizedSignomial):
        objectiveClassificationOrig = "generalized signomial";
        break;

    case(E_ObjectiveFunctionClassification::Nonalgebraic):
        objectiveClassificationOrig = "nonalgebraic";
        break;

    default:
        objectiveClassificationOrig = "unknown";
        break;
    }

    switch(static_cast<E_Convexity>(env->problem->objectiveFunction->properties.convexity))
    {
    case(E_Convexity::Linear):
        break;

    case(E_Convexity::Convex):
        objectiveClassificationOrig += ", convex";
        break;

    case(E_Convexity::Concave):
        objectiveClassificationOrig += ", concave";
        break;

    case(E_Convexity::Nonconvex):
        objectiveClassificationOrig += ", nonconvex";
        break;

    default:
        break;
    }

    if(isReformulated)
    {
        objectiveDirectionRef
            = env->reformulatedProblem->objectiveFunction->properties.isMinimize ? "minimize" : "maximize";

        switch(static_cast<E_ObjectiveFunctionClassification>(
            env->reformulatedProblem->objectiveFunction->properties.classification))
        {

        case(E_ObjectiveFunctionClassification::Linear):
            objectiveClassificationRef = "linear";
            break;

        case(E_ObjectiveFunctionClassification::Quadratic):
            objectiveClassificationRef = "quadratic";
            break;

        case(E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear):
            objectiveClassificationRef = "quadratic but considered as nonlinear";
            break;

        case(E_ObjectiveFunctionClassification::Signomial):
            objectiveClassificationRef = "signomial";
            break;

        case(E_ObjectiveFunctionClassification::Nonlinear):
            objectiveClassificationRef = "nonlinear";
            break;

        case(E_ObjectiveFunctionClassification::GeneralizedSignomial):
            objectiveClassificationRef = "generalized signomial";
            break;

        case(E_ObjectiveFunctionClassification::Nonalgebraic):
            objectiveClassificationRef = "nonalgebraic";
            break;

        default:
            objectiveClassificationRef = "unknown";
            break;
        }

        switch(static_cast<E_Convexity>(env->reformulatedProblem->objectiveFunction->properties.convexity))
        {
        case(E_Convexity::Linear):
            break;

        case(E_Convexity::Convex):
            objectiveClassificationRef += ", convex";
            break;

        case(E_Convexity::Concave):
            objectiveClassificationRef += ", concave";
            break;

        case(E_Convexity::Nonconvex):
            objectiveClassificationRef += ", nonconvex";
            break;

        default:
            break;
        }
    }

    report << "\n";

    report << fmt::format(
                  " {:35s}{:21s}{:s}", "Objective function direction:", objectiveDirectionOrig, objectiveDirectionRef)
           << "\n";

    report << fmt::format(" {:35s}{:21s}{:s}", "Objective function type:", objectiveClassificationOrig,
                  objectiveClassificationRef)
           << "\n";

    report << "\n";

    if(isReformulated)
    {
        if(env->problem->properties.numberOfNumericConstraints > 0
            || env->reformulatedProblem->properties.numberOfNumericConstraints > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          "Number of constraints:", env->problem->properties.numberOfNumericConstraints,
                          env->reformulatedProblem->properties.numberOfNumericConstraints)
                   << "\n";

        if(env->problem->properties.numberOfLinearConstraints > 0
            || env->reformulatedProblem->properties.numberOfLinearConstraints > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          " - linear:", env->problem->properties.numberOfLinearConstraints,
                          env->reformulatedProblem->properties.numberOfLinearConstraints)

                   << "\n";

        if(env->problem->properties.numberOfConvexQuadraticConstraints > 0
            || env->reformulatedProblem->properties.numberOfConvexQuadraticConstraints > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          " - convex quadratic:", env->problem->properties.numberOfConvexQuadraticConstraints,
                          env->reformulatedProblem->properties.numberOfConvexQuadraticConstraints)

                   << "\n";

        if(env->problem->properties.numberOfNonconvexQuadraticConstraints > 0
            || env->reformulatedProblem->properties.numberOfNonconvexQuadraticConstraints > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          " - nonconvex quadratic:", env->problem->properties.numberOfNonconvexQuadraticConstraints,
                          env->reformulatedProblem->properties.numberOfNonconvexQuadraticConstraints)

                   << "\n";

        if(env->problem->properties.numberOfConvexNonlinearConstraints > 0
            || env->reformulatedProblem->properties.numberOfConvexNonlinearConstraints > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          " - convex nonlinear:", env->problem->properties.numberOfConvexNonlinearConstraints,
                          env->reformulatedProblem->properties.numberOfConvexNonlinearConstraints)
                   << "\n";

        if(env->problem->properties.numberOfNonconvexNonlinearConstraints > 0
            || env->reformulatedProblem->properties.numberOfNonconvexNonlinearConstraints > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          " - nonconvex nonlinear:", env->problem->properties.numberOfNonconvexNonlinearConstraints,
                          env->reformulatedProblem->properties.numberOfNonconvexNonlinearConstraints)
                   << "\n";
    }
    else
    {
        if(env->problem->properties.numberOfNumericConstraints > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          "Number of constraints:", env->problem->properties.numberOfNumericConstraints, "")
                   << "\n";

        if(env->problem->properties.numberOfLinearConstraints > 0)
            report << fmt::format(
                          " {:35s}{:<21d}{:d}", " - linear:", env->problem->properties.numberOfLinearConstraints, "")
                   << "\n";

        if(env->problem->properties.numberOfQuadraticConstraints > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          " - quadratic:", env->problem->properties.numberOfQuadraticConstraints, "")
                   << "\n";

        if(env->problem->properties.numberOfNonlinearConstraints > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          " - nonlinear:", env->problem->properties.numberOfNonlinearConstraints, "")
                   << "\n";
    }

    report << "\n";

    if(isReformulated)
    {
        if(env->problem->properties.numberOfVariables > 0 || env->reformulatedProblem->properties.numberOfVariables > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          "Number of variables:", env->problem->properties.numberOfVariables,
                          env->reformulatedProblem->properties.numberOfVariables)
                   << "\n";

        if(env->problem->properties.numberOfRealVariables > 0
            || env->reformulatedProblem->properties.numberOfRealVariables > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}", " - real:", env->problem->properties.numberOfRealVariables,
                          env->reformulatedProblem->properties.numberOfRealVariables)
                   << "\n";

        if(env->problem->properties.numberOfBinaryVariables > 0
            || env->reformulatedProblem->properties.numberOfBinaryVariables > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}", " - binary:", env->problem->properties.numberOfBinaryVariables,
                          env->reformulatedProblem->properties.numberOfBinaryVariables)
                   << "\n";

        if(env->problem->properties.numberOfIntegerVariables > 0
            || env->reformulatedProblem->properties.numberOfIntegerVariables > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          " - integer:", env->problem->properties.numberOfIntegerVariables,
                          env->reformulatedProblem->properties.numberOfIntegerVariables)
                   << "\n";

        if(env->problem->properties.numberOfSemicontinuousVariables > 0
            || env->reformulatedProblem->properties.numberOfSemicontinuousVariables > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          " - semicontinuous:", env->problem->properties.numberOfSemicontinuousVariables,
                          env->reformulatedProblem->properties.numberOfSemicontinuousVariables)
                   << "\n";

        if(env->problem->properties.numberOfNonlinearVariables > 0
            || env->reformulatedProblem->properties.numberOfNonlinearVariables > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          " - nonlinear:", env->problem->properties.numberOfNonlinearVariables,
                          env->reformulatedProblem->properties.numberOfNonlinearVariables)
                   << "\n";
    }
    else
    {
        if(env->problem->properties.numberOfVariables > 0)
            report << fmt::format(
                          " {:35s}{:<21d}{:d}", "Number of variables:", env->problem->properties.numberOfVariables, "")
                   << "\n";

        if(env->problem->properties.numberOfRealVariables > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}", " - real:", env->problem->properties.numberOfRealVariables, "")
                   << "\n";

        if(env->problem->properties.numberOfBinaryVariables > 0)
            report << fmt::format(
                          " {:35s}{:<21d}{:d}", " - binary:", env->problem->properties.numberOfBinaryVariables, "")
                   << "\n";

        if(env->problem->properties.numberOfIntegerVariables > 0)
            report << fmt::format(
                          " {:35s}{:<21d}{:d}", " - integer:", env->problem->properties.numberOfIntegerVariables, "")
                   << "\n";

        if(env->problem->properties.numberOfSemicontinuousVariables > 0)
            report << fmt::format(" {:35s}{:<21d}{:d}",
                          " - semicontinuous:", env->problem->properties.numberOfSemicontinuousVariables, "")
                   << "\n";
    }

    if(env->results->auxiliaryVariablesIntroduced.size() > 0)
    {
        int totalNumberOfTransformations = 0;

        for(auto& AUXVAR : env->results->auxiliaryVariablesIntroduced)
            totalNumberOfTransformations += AUXVAR.second;

        if(env->problem->properties.numberOfVariables > 0)
            report << '\n';
        report << fmt::format(" {:56s}{:d}", "Number of transformations performed:", totalNumberOfTransformations)
               << "\n";

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::NonlinearObjectiveFunction);
            value > 0)
            report << fmt::format(" {:56s}{:d}", "- epigraph:", "", value) << "\n";

        if(auto value
            = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::NonlinearExpressionPartitioning);
            value > 0)
            report << fmt::format(" {:56s}{:d}", "- nonlinear expression partitioning:", value) << "\n";

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::MonomialTermsPartitioning);
            value > 0)
            report << fmt::format(" {:56s}{:d}", "- monomial terms partitioning:", value) << "\n";

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::SignomialTermsPartitioning);
            value > 0)
            report << fmt::format(" {:56s}{:d}", "- signomial terms partitioning:", value) << "\n";

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::ContinuousBilinear);
            value > 0)
            report << fmt::format(" {:56s}{:d}", "- continuous bilinear term extraction:", value) << "\n";

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::BinaryBilinear); value > 0)
            report << fmt::format(" {:56s}{:d}", "- binary bilinear term reformulation:", value) << "\n";

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::BinaryContinuousBilinear);
            value > 0)
            report << fmt::format(" {:56s}{:d}", "- binary/continuous bilinear term reformulation:", value) << "\n";

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::IntegerBilinear); value > 0)
            report << fmt::format(" {:56s}{:d}", "- integer bilinear term reformulation:", value) << "\n";

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::BinaryMonomial); value > 0)
            report << fmt::format(" {:56s}{:d}", "- binary monomial term reformulation:", value) << "\n";

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::AbsoluteValue); value > 0)
            report << fmt::format(" {:56s}{:d}", "-absolute value reformulation:", value) << "\n";
    }

    env->output->outputInfo(report.str());
}

void Report::outputSolutionReport()
{
    std::stringstream report;

    report << "\n\n";

#ifdef SIMPLE_OUTPUT_CHARS
    report << "- Solution report ";
    report
        << "-----------------------------------------------------------------------------------------------------\n";
#else
    report << "╶ Solution report ";
    report
        << "────────────────────────────────────────────────────────────────────────────────────────────────────╴\n";
#endif

    report << "\n";

    bool primalSolutionFound = env->results->hasPrimalSolution();

    if(env->results->terminationReasonDescription != "")
        report << " " << env->results->terminationReasonDescription << "\n\n";

    switch(env->results->getModelReturnStatus())
    {
    /* case E_ModelReturnStatus::OptimalLocal:
        report << " Optimal primal solution found, but it might be local since the model seems to be nonconvex.\n";
        break;*/
    case E_ModelReturnStatus::OptimalGlobal:
        report << " Globally optimal primal solution found.\n";
        break;
    case E_ModelReturnStatus::FeasibleSolution:
        if(env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Convex)
            report << " Feasible primal solution found to convex problem. Can not guarantee optimality to the given "
                      "termination criteria.\n";
        else
            report << " Feasible primal solution found to nonconvex problem. Can not guarantee optimality to the given "
                      "termination criteria.\n";
        break;
    case E_ModelReturnStatus::InfeasibleLocal:
        report << " Problem found to be infeasible, but globality could not be verified since the problem seems to be "
                  "nonconvex.\n";
        break;
    case E_ModelReturnStatus::InfeasibleGlobal:
        report << " Problem is infeasible.\n";
        break;
    case E_ModelReturnStatus::Unbounded:
        report << " Problem is unbounded, but a primal solution was found.\n";
        break;
    case E_ModelReturnStatus::UnboundedNoSolution:
        report << " Problem is unbounded, and no primal solution was found.\n";
        break;
    case E_ModelReturnStatus::NoSolutionReturned:
        report << " No solution found. Try modifying the termination criteria.\n";
        break;
    case E_ModelReturnStatus::ErrorUnknown:
        report << " An error occurred, but a primal solution was found.\n";
        break;
    case E_ModelReturnStatus::None:
    case E_ModelReturnStatus::ErrorNoSolution:
        report << " An error occurred, and no primal solution was found.\n";
    };

    // Warn the user if variables are at maximum bound limits

    if(primalSolutionFound)
    {
        bool variablesAreBounded = true;
        double minLBCont = env->settings->getSetting<double>("Variables.Continuous.MinimumLowerBound", "Model");
        double maxUBCont = env->settings->getSetting<double>("Variables.Continuous.MaximumUpperBound", "Model");
        double minLBInt = env->settings->getSetting<double>("Variables.Integer.MinimumLowerBound", "Model");
        double maxUBInt = env->settings->getSetting<double>("Variables.Integer.MaximumUpperBound", "Model");

        if(minLBInt == 0)
            minLBInt = -maxUBInt; // In case a min lower bound of zero is used, we do not want to give false warnings

        for(auto& V : env->problem->realVariables)
        {
            if(env->results->primalSolution.at(V->index) == minLBCont
                || env->results->primalSolution.at(V->index) == maxUBCont)
            {
                variablesAreBounded = false;
                break;
            }
        }

        if(variablesAreBounded)
        {
            for(auto& V : env->problem->integerVariables)
            {
                if(env->results->primalSolution.at(V->index) == minLBInt
                    || env->results->primalSolution.at(V->index) == maxUBInt)
                {
                    variablesAreBounded = false;
                    break;
                }
            }
        }

        if(!variablesAreBounded)
            report
                << " Warning! Solution point is at maximum variable bounds. Problem might be artificially bounded.\n";
    }

    report << "\n";

    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        report << " Objective bound (minimization) [dual, primal]:  ";
        report << "[" << Utilities::toStringFormat(env->results->getGlobalDualBound(), "{:g}") << ", ";
        report << Utilities::toStringFormat(env->results->getPrimalBound(), "{:g}") << "]\n";
    }
    else
    {
        report << " Objective bound (maximization) [primal, dual]:  ";
        report << "[" << Utilities::toStringFormat(env->results->getPrimalBound(), "{:g}") << ", ";
        report << Utilities::toStringFormat(env->results->getGlobalDualBound(), "{:g}") << "]\n";
    }

    report << " Objective gap absolute / relative:              ";
    report << "" << Utilities::toStringFormat(env->results->getAbsoluteGlobalObjectiveGap(), "{:g}") << " / ";
    report << Utilities::toStringFormat(env->results->getRelativeGlobalObjectiveGap(), "{:g}") << "\n";
    report << "\n";

    std::stringstream fulfilled;
    std::stringstream unfulfilled;

    if(env->results->isAbsoluteObjectiveGapToleranceMet())
    {
        fulfilled << "  - absolute objective gap tolerance             ";
        fulfilled << env->results->getAbsoluteGlobalObjectiveGap() << " <= ";
        fulfilled << env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") << "\n";
    }
    else
    {
        unfulfilled << "  - absolute objective gap tolerance             ";
        unfulfilled << env->results->getAbsoluteGlobalObjectiveGap() << " > ";
        unfulfilled << env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") << "\n";
    }

    if(env->results->isRelativeObjectiveGapToleranceMet())
    {
        fulfilled << "  - relative objective gap tolerance             ";
        fulfilled << env->results->getRelativeGlobalObjectiveGap() << " <= ";
        fulfilled << env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination") << "\n";
    }
    else
    {
        unfulfilled << "  - relative objective gap tolerance             ";
        unfulfilled << env->results->getRelativeGlobalObjectiveGap() << " > ";
        unfulfilled << env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination") << "\n";
    }

    if(static_cast<ES_TreeStrategy>(env->settings->getSetting<int>("TreeStrategy", "Dual"))
        != ES_TreeStrategy::SingleTree)
    {
        if(env->results->getCurrentIteration()->maxDeviation
            <= env->settings->getSetting<double>("ConstraintTolerance", "Termination"))
        {
            fulfilled << "  - maximal constraint tolerance                 ";
            fulfilled << env->results->getCurrentIteration()->maxDeviation << " <= ";
            fulfilled << env->settings->getSetting<double>("ConstraintTolerance", "Termination") << "\n";
        }
        else
        {
            unfulfilled << "  - maximal constraint tolerance                 ";
            unfulfilled << env->results->getCurrentIteration()->maxDeviation << " > ";
            unfulfilled << env->settings->getSetting<double>("ConstraintTolerance", "Termination") << "\n";
        }
    }

    int iterLim = env->settings->getSetting<int>("IterationLimit", "Termination");

    if(env->results->getCurrentIteration()->iterationNumber > iterLim)
    {
        fulfilled << "  - iteration limit                              ";
        fulfilled << env->results->getCurrentIteration()->iterationNumber << " > ";
        fulfilled << iterLim << "\n";
    }
    else
    {
        unfulfilled << "  - iteration limit                              ";
        unfulfilled << env->results->getCurrentIteration()->iterationNumber << " <= ";
        unfulfilled << iterLim << "\n";
    }

    if(env->timing->getElapsedTime("Total") > env->settings->getSetting<double>("TimeLimit", "Termination"))
    {
        fulfilled << "  - solution time limit (s)                      ";
        fulfilled << env->timing->getElapsedTime("Total") << " > ";
        fulfilled << env->settings->getSetting<double>("TimeLimit", "Termination") << "\n";
    }
    else
    {
        unfulfilled << "  - solution time limit (s)                      ";
        unfulfilled << env->timing->getElapsedTime("Total") << " <= ";
        unfulfilled << env->settings->getSetting<double>("TimeLimit", "Termination") << "\n";
    }

    report << " Fulfilled termination criteria: \n";
    report << fulfilled.str();
    report << "\n";

    report << " Unfulfilled termination criteria: \n";
    report << unfulfilled.str();
    report << "\n";

    report << " Dual problems solved in main step:              ";
    report << env->solutionStatistics.getNumberOfTotalDualProblems() << "\n";

    if(env->solutionStatistics.numberOfProblemsLP > 0)
    {
        report << "  - LP problems                                  " << env->solutionStatistics.numberOfProblemsLP
               << "\n";
    }

    if(env->solutionStatistics.numberOfProblemsQP > 0)
    {
        report << "  - QP problems                                  " << env->solutionStatistics.numberOfProblemsQP
               << "\n";
    }

    if(env->solutionStatistics.numberOfProblemsQCQP > 0)
    {
        report << "  - QCQP problems                                " << env->solutionStatistics.numberOfProblemsQCQP
               << "\n";
    }

    if(env->solutionStatistics.numberOfProblemsOptimalMILP > 0)
    {
        report << "  - MILP problems, optimal                       "
               << env->solutionStatistics.numberOfProblemsOptimalMILP << "\n";
    }

    if(env->solutionStatistics.numberOfProblemsFeasibleMILP > 0)
    {
        report << "  - MILP problems, feasible                      "
               << env->solutionStatistics.numberOfProblemsFeasibleMILP << "\n";
    }

    if(env->solutionStatistics.numberOfProblemsOptimalMIQP > 0)
    {
        report << "  - MIQP problems, optimal                       "
               << env->solutionStatistics.numberOfProblemsOptimalMIQP << "\n";
    }

    if(env->solutionStatistics.numberOfProblemsFeasibleMIQP > 0)
    {
        report << "  - MIQP problems, feasible                      "
               << env->solutionStatistics.numberOfProblemsFeasibleMIQP << "\n";
    }

    report << "\n";

    if(env->solutionStatistics.numberOfExploredNodes > 0)
    {
        report << " Number of explored nodes:                       ";
        report << env->solutionStatistics.numberOfExploredNodes << "\n";

        report << "\n";
    }

    if(env->solutionStatistics.numberOfProblemsNLPInteriorPointSearch > 0
        || env->solutionStatistics.numberOfProblemsMinimaxLP > 0)
    {
        report << " Problems solved during interior point search: \n";

        if(env->solutionStatistics.numberOfProblemsNLPInteriorPointSearch > 0)
        {
            report << " - NLP problems:                                 ";
            report << env->solutionStatistics.numberOfProblemsNLPInteriorPointSearch << "\n";
        }

        if(env->solutionStatistics.numberOfProblemsMinimaxLP > 0)
        {
            report << " - LP problems:                                  ";
            report << env->solutionStatistics.numberOfProblemsMinimaxLP << "\n";
        }

        report << "\n";
    }

    if(env->solutionStatistics.numberOfProblemsFixedNLP > 0)
    {
        report << " Fixed primal NLP problems solved:               ";
        report << env->solutionStatistics.numberOfProblemsFixedNLP << "\n";
        report << "\n";
    }

    if(env->results->hasPrimalSolution())
    {
        report << fmt::format(" {:<48}{:d}",
                      "Number of primal solutions found:", env->solutionStatistics.numberOfFoundPrimalSolutions)
               << "\n";

        for(auto& S : env->results->primalSolutionSourceStatistics)
        {
            std::string sourceDesc;

            switch(S.first)
            {
            case E_PrimalSolutionSource::Rootsearch:
                sourceDesc = "root search";
                break;
            case E_PrimalSolutionSource::RootsearchFixedIntegers:
                sourceDesc = "root search with fixed integers";
                break;
            case E_PrimalSolutionSource::NLPFixedIntegers:
                sourceDesc = "NLP problem with fixed integers";
                break;
            case E_PrimalSolutionSource::MIPSolutionPool:
                sourceDesc = "MILP solution pool";
                break;
            case E_PrimalSolutionSource::LPFixedIntegers:
                sourceDesc = "LP problem with fixed integers";
                break;
            case E_PrimalSolutionSource::LazyConstraintCallback:
                sourceDesc = "lazy constraint callback";
                break;
            case E_PrimalSolutionSource::HeuristicCallback:
                sourceDesc = "heuristic constraint callback";
                break;
            case E_PrimalSolutionSource::IncumbentCallback:
                sourceDesc = "incumbent constraint callback";
                break;
            default:
                sourceDesc = "other";
                break;
            }

            report << fmt::format(" - {:<46}{:d}", sourceDesc + ':', S.second) << "\n";
        }

        report << "\n";
    }

    for(auto& T : env->timing->timers)
    {
        T.stop();
        auto elapsed = T.elapsed();

        if(elapsed > 0)
        {
            report << fmt::format(" {:<48}{:g}", T.description + ':', elapsed) << "\n";
        }
    }

    env->output->outputInfo(report.str());
}

void Report::outputInteriorPointPreReport()
{
    std::stringstream report;

#ifdef SIMPLE_OUTPUT_CHARS
    report << "\n";
    report << "- Interior point search ";
    report << "-----------------------------------------------------------------------------------------------\n";
#else
    report << "\n";
    report << "╶ Interior point search ";
    report << "──────────────────────────────────────────────────────────────────────────────────────────────╴\n";
#endif

    report << "\n";

    report << " Strategy selected:          ";

    switch(static_cast<ES_InteriorPointStrategy>(env->settings->getSetting<int>("ESH.InteriorPoint.Solver", "Dual")))
    {
    case(ES_InteriorPointStrategy::CuttingPlaneMiniMax):
        report << "cutting plane minimax";
        break;
    case(ES_InteriorPointStrategy::IpoptMinimax):
        report << "Ipopt minimax";
        break;
    case(ES_InteriorPointStrategy::IpoptRelaxed):
        report << "Ipopt relaxed";
        break;
    case(ES_InteriorPointStrategy::IpoptMinimaxAndRelaxed):
        report << "Ipopt minimax and Ipopt relaxed";
        break;
    default:
        report << "none";
        break;
    }

    env->output->outputInfo(report.str());
}
} // namespace SHOT