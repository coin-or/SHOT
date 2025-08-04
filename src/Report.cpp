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

Report::Report(EnvironmentPtr envPtr) : env(envPtr) { }

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
            env->output->outputInfo("");
            this->outputIterationDetailHeader();
        }

        iterationsWithoutPrintoutCounter = 0;
        iterationPrintoutsSinceLastHeader++;
        lastIterationOutputTimeStamp = env->timing->getElapsedTime("Total");

        std::string combDualCuts = "";

        if(lineType == E_IterationLineType::DualRepair)
        {
            combDualCuts = fmt::format("Relaxed constraints: {:<4d}", dualCutsAdded);
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
            env->output->outputDebug("");
            env->output->outputInfo(fmt::format("{:>6d}: {:<10s}{:^10.2f}{:^13s}{:>27s}{:>19s}{:<32s}", iterationNumber,
                iterationDesc, totalTime, combDualCuts, "", "", ""));
            env->output->outputDebug("");

            env->output->outputInfo(fmt::format("{} {} {} / {}", std::string(27, ' '),
                "Total primal improvements after repair / reduction cut:  ",
                env->solutionStatistics.numberOfPrimalImprovementsAfterInfeasibilityRepair,
                env->solutionStatistics.numberOfPrimalImprovementsAfterReductionCut));
        }
        else
        {
            env->output->outputDebug("");
            env->output->outputInfo(fmt::format("{:>6d}: {:<10s}{:^10.2f}{:>13s}{:>27s}{:>19s}{:<32s}", iterationNumber,
                iterationDesc, totalTime, combDualCuts, combObjectiveValue, combObjectiveGap, combCurrSol));
            env->output->outputDebug("");
        }

        if(env->results->getCurrentIteration()->numberOfExploredNodes > 0
            || env->results->getCurrentIteration()->numberOfOpenNodes > 0)
        {
            env->output->outputDebug(fmt::format("        Explored nodes: {}. Open nodes: {}.",
                env->solutionStatistics.numberOfExploredNodes, env->results->getCurrentIteration()->numberOfOpenNodes));
        }
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

        env->output->outputInfo(fmt::format("{:6d}: {:<10s}{:^10.2f}{:13s}{:27s}{:19s}", iterationNumber, iterationDesc,
            totalTime, combDualCuts, combObjectiveValue, combObjectiveGap));
    }
    catch(...)
    {
        env->output->outputError("Cannot write iteration solution report!");
    }
}

void Report::outputIterationDetailHeader()
{
    firstIterationHeaderPrinted = true;

#ifdef SIMPLE_OUTPUT_CHARS
    env->output->outputInfo(
        "    Iteration     |  Time  |  Dual cuts  |     Objective value     |   Objective gap   |     Current "
        "solution");

    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        env->output->outputInfo(
            "     #: type      |  tot.  |   + | tot.  |       dual | primal     |    abs. | rel.    |    obj.fn. | "
            "max.err.");
    }
    else
    {
        env->output->outputInfo(
            "     #: type      |  tot.  |   + | tot.  |     primal | dual       |    abs. | rel.    |    obj.fn. | "
            "max.err.");
    }

    env->output->outputInfo(
        "----------------------------------------------------------------------------------------------------"
        "----------------");
#else
    env->output->outputInfo(
        "    Iteration     │  Time  │  Dual cuts  │     Objective value     │   Objective gap   │     Current "
        "solution");

    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        env->output->outputInfo(
            "     #: type      │  tot.  │   + | tot.  │       dual | primal     │    abs. | rel.    │    obj.fn. | "
            "max.err.");
    }
    else
    {
        env->output->outputInfo(
            "     #: type      │  tot.  │   + | tot.  │     primal | dual       │    abs. | rel.    │    obj.fn. | "
            "max.err.)");
    }

    env->output->outputInfo("╶─────────────────┴────────┴─────────────┴─────────────────────────┴───────────────────┴──"
                            "────────────────────────────╴");
#endif

    env->output->outputInfo("");
    iterationPrintoutsSinceLastHeader = 0;
}

void Report::outputIterationDetailHeaderMinimax()
{

#ifdef SIMPLE_OUTPUT_CHARS
    env->output->outputInfo(
        "    Iteration      |  Time  |    Cuts     |     Objective value     |  Objective diff.   ");
    env->output->outputInfo("     #: type       |  tot.  |   + | tot.  |    problem | line srch  |    abs. | rel.    ");
    env->output->outputInfo("---------------------------------------------------------------------------------------");
#else
    env->output->outputInfo("    Iteration     │  Time  │    Cuts     │     Objective value     │  Objective diff.   ");
    env->output->outputInfo("     #: type      │  tot.  │   + | tot.  │    problem | line srch  │    abs. | rel.    ");
    env->output->outputInfo("╶─────────────────┴────────┴─────────────┴─────────────────────────┴──────────────────╴");
#endif
}

void Report::outputSolverHeader()
{
#ifdef SIMPLE_OUTPUT_CHARS
    env->output->outputInfo("");
    env->output->outputInfo("- Supporting Hyperplane Optimization Toolkit (SHOT) "
                            "-------------------------------------------------------------------");
#else
    env->output->outputInfo("");
    env->output->outputInfo("╶ Supporting Hyperplane Optimization Toolkit (SHOT) "
                            "──────────────────────────────────────────────────────────────────╴");
#endif

    env->output->outputInfo("");

    env->output->outputInfo(fmt::format(" Andreas Lundell and Jan Kronqvist, Åbo Akademi University, Finland."));
    env->output->outputInfo(" See documentation for full list of contributors and utilized software libraries.");

    env->output->outputInfo("");

    if(SHOT_VERSION_PATCH[0] != '0')
    {
        env->output->outputInfo(fmt::format(" Version: {}.{}.{}. Git hash: {}. Released: {}.", SHOT_VERSION_MAJOR,
            SHOT_VERSION_MINOR, SHOT_VERSION_PATCH, SHOT_GITHASH, __DATE__));
    }
    else
    {
        env->output->outputInfo(fmt::format(" Version: {}.{}. Git hash: {}. Released: {}.", SHOT_VERSION_MAJOR,
            SHOT_VERSION_MINOR, SHOT_GITHASH, __DATE__));
    }

    env->output->outputInfo("");
    env->output->outputInfo(" For more information visit https://shotsolver.dev");
    env->output->outputInfo("");
}

void Report::outputOptionsReport()
{
    env->output->outputInfo("");

#ifdef SIMPLE_OUTPUT_CHARS
    env->output->outputInfo("- Options "
                            "------------------------------------------------------------------------------------------"
                            "-------------------");
#else
    env->output->outputInfo("╶ Options "
                            "──────────────────────────────────────────────────────────────────────────────────────────"
                            "──────────────────╴");
#endif

    env->output->outputInfo("");

    if(auto optionsFile = env->settings->getSetting<std::string>("OptionsFile", "Input"); optionsFile == "")
        env->output->outputInfo(" No options file specified.");
    else
        env->output->outputInfo(fmt::format(" Options read from file:     {}", optionsFile));

    env->output->outputInfo("");

    if(auto nonDefaultSettings = env->settings->getChangedSettings(); nonDefaultSettings.size() > 0)
    {
        env->output->outputInfo(" Options specified:");
        env->output->outputInfo("");

        for(auto& S : nonDefaultSettings)
            env->output->outputInfo(fmt::format("  - {}", S));

        env->output->outputInfo("");
    }

    std::string cutAlgorithm, dualSolver;

    if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::ESH)
    {
        cutAlgorithm = "ESH";
    }
    else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::ECP)
    {
        cutAlgorithm = "ECP";
    }
    else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::OnlyExternal)
    {
        cutAlgorithm = "Only external cuts";
    }
    else
    {
        cutAlgorithm = "Unknown";
    }

    auto solver = static_cast<ES_MIPSolver>(env->settings->getSetting<int>("MIP.Solver", "Dual"));

#ifdef HAS_CPLEX
    if(solver == ES_MIPSolver::Cplex)
        dualSolver = "CPLEX";
#endif

#ifdef HAS_GUROBI
    if(solver == ES_MIPSolver::Gurobi)
        dualSolver = "Gurobi";
#endif

#ifdef HAS_CBC
    if(solver == ES_MIPSolver::Cbc)
        dualSolver = "Cbc";
#endif

    switch(static_cast<E_SolutionStrategy>(env->results->usedSolutionStrategy))
    {
    case(E_SolutionStrategy::SingleTree):
        env->output->outputInfo(" Dual strategy:              Single-tree");
        env->output->outputInfo(fmt::format("  - cut algorithm:           {}", cutAlgorithm));
        break;
    case(E_SolutionStrategy::MultiTree):
        env->output->outputInfo(" Dual strategy:              Multi-tree");
        env->output->outputInfo(fmt::format("  - cut algorithm:           {}", cutAlgorithm));
        break;
    case(E_SolutionStrategy::NLP):
        env->output->outputInfo(" Dual strategy:              NLP version");
        env->output->outputInfo(fmt::format("  - cut algorithm:           {}", cutAlgorithm));
        break;
    case(E_SolutionStrategy::MIQP):
        env->output->outputInfo(" Dual strategy:              MIQP version");
        break;
    case(E_SolutionStrategy::MIQCQP):
        env->output->outputInfo(" Dual strategy:              MIQCQP version");
        break;
    default:
        break;
    }

    env->output->outputInfo(
        fmt::format("  - solver:                  {} {}", dualSolver, env->dualSolver->MIPSolver->getSolverVersion()));

    env->output->outputInfo("");

    env->output->outputInfo(fmt::format(" Primal NLP solver:          {}",
        (static_cast<ES_PrimalNLPSolver>(env->results->usedPrimalNLPSolver) == ES_PrimalNLPSolver::None)
            ? "none"
            : env->results->usedPrimalNLPSolverDescription));

    env->output->outputInfo("");

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        env->output->outputInfo(fmt::format(
            " Debug directory:            {}", env->settings->getSetting<std::string>("Debug.Path", "Output")));
}

void Report::outputModelingSystemReport(ES_SourceFormat source, std::string filename)
{
#ifdef SIMPLE_OUTPUT_CHARS
    env->output->outputInfo(
        "- Modeling system "
        "-----------------------------------------------------------------------------------------------------");
#else
    env->output->outputInfo(
        "╶ Modeling system "
        "────────────────────────────────────────────────────────────────────────────────────────────────────╴");
#endif

    env->output->outputInfo("");

    switch(source)
    {
    case(ES_SourceFormat::GAMS):
        env->output->outputInfo(" Modeling system:            GAMS");
        break;
    case(ES_SourceFormat::OSiL):
        env->output->outputInfo(" Modeling system:            OSiL");
        break;
    case(ES_SourceFormat::NL):
        env->output->outputInfo(" Modeling system:            AMPL");
        break;

    default:
        break;
    }

    if(filename != "")
        env->output->outputInfo(fmt::format(" Problem read from file:     {}", filename));

    env->output->outputInfo("");
}

void Report::outputProblemInstanceReport()
{
    bool isReformulated = (env->problem == env->reformulatedProblem) ? false : true;

    env->output->outputInfo("");

#ifdef SIMPLE_OUTPUT_CHARS
    env->output->outputInfo(
        "- Problem instance "
        "----------------------------------------------------------------------------------------------------");
#else
    env->output->outputInfo(
        "╶ Problem instance "
        "───────────────────────────────────────────────────────────────────────────────────────────────────╴");
#endif

    env->output->outputInfo("");

    if(isReformulated)
        env->output->outputInfo(fmt::format(" {:35s}{:21s}{:s}", "", "Original", "Reformulated"));
    else
        env->output->outputInfo(fmt::format(" {:35s}{:21s}{:s}", "", "Original", ""));

    env->output->outputInfo("");

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

    env->output->outputInfo(fmt::format(
        " {:35s}{:21s}{:s}", "Problem classification:", problemClassificationOrig, problemClassificationRef));

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

    env->output->outputInfo("");

    env->output->outputInfo(fmt::format(
        " {:35s}{:21s}{:s}", "Objective function direction:", objectiveDirectionOrig, objectiveDirectionRef));

    env->output->outputInfo(fmt::format(
        " {:35s}{:21s}{:s}", "Objective function type:", objectiveClassificationOrig, objectiveClassificationRef));

    env->output->outputInfo("");

    if(isReformulated)
    {
        if(env->problem->properties.numberOfNumericConstraints > 0
            || env->reformulatedProblem->properties.numberOfNumericConstraints > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:d}",
                "Number of constraints:", env->problem->properties.numberOfNumericConstraints,
                env->reformulatedProblem->properties.numberOfNumericConstraints));

        if(env->problem->properties.numberOfLinearConstraints > 0
            || env->reformulatedProblem->properties.numberOfLinearConstraints > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:d}", " - linear:",
                env->problem->properties.numberOfLinearConstraints
                    - env->problem->properties.numberOfAddedLinearizations,
                env->reformulatedProblem->properties.numberOfLinearConstraints
                    - env->reformulatedProblem->properties.numberOfAddedLinearizations));

        if(env->problem->properties.numberOfConvexQuadraticConstraints > 0
            || env->reformulatedProblem->properties.numberOfConvexQuadraticConstraints > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:d}",
                " - convex quadratic:", env->problem->properties.numberOfConvexQuadraticConstraints,
                env->reformulatedProblem->properties.numberOfConvexQuadraticConstraints));

        if(env->problem->properties.numberOfNonconvexQuadraticConstraints > 0
            || env->reformulatedProblem->properties.numberOfNonconvexQuadraticConstraints > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:d}",
                " - nonconvex quadratic:", env->problem->properties.numberOfNonconvexQuadraticConstraints,
                env->reformulatedProblem->properties.numberOfNonconvexQuadraticConstraints));

        if(env->problem->properties.numberOfConvexNonlinearConstraints > 0
            || env->reformulatedProblem->properties.numberOfConvexNonlinearConstraints > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:d}",
                " - convex nonlinear:", env->problem->properties.numberOfConvexNonlinearConstraints,
                env->reformulatedProblem->properties.numberOfConvexNonlinearConstraints));

        if(env->problem->properties.numberOfNonconvexNonlinearConstraints > 0
            || env->reformulatedProblem->properties.numberOfNonconvexNonlinearConstraints > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:d}",
                " - nonconvex nonlinear:", env->problem->properties.numberOfNonconvexNonlinearConstraints,
                env->reformulatedProblem->properties.numberOfNonconvexNonlinearConstraints));

        if(env->problem->properties.numberOfAddedLinearizations > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:d}",
                " - added linearizations:", env->problem->properties.numberOfAddedLinearizations,
                env->reformulatedProblem->properties.numberOfAddedLinearizations));
    }
    else
    {
        if(env->problem->properties.numberOfNumericConstraints > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:s}", "Number of constraints:",
                env->problem->properties.numberOfNumericConstraints
                    - env->problem->properties.numberOfAddedLinearizations,
                ""));

        if(env->problem->properties.numberOfLinearConstraints > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:s}", " - linear:",
                env->problem->properties.numberOfLinearConstraints
                    - env->problem->properties.numberOfAddedLinearizations,
                ""));

        if(env->problem->properties.numberOfQuadraticConstraints > 0)
            env->output->outputInfo(fmt::format(
                " {:35s}{:<21d}{:s}", " - quadratic:", env->problem->properties.numberOfQuadraticConstraints, ""));

        if(env->problem->properties.numberOfNonlinearConstraints > 0)
            env->output->outputInfo(fmt::format(
                " {:35s}{:<21d}{:s}", " - nonlinear:", env->problem->properties.numberOfNonlinearConstraints, ""));

        if(env->problem->properties.numberOfNonlinearConstraints > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:s}",
                " - added linearizations:", env->problem->properties.numberOfAddedLinearizations, ""));
    }

    env->output->outputInfo("");

    if(isReformulated)
    {
        if(env->problem->properties.numberOfVariables > 0 || env->reformulatedProblem->properties.numberOfVariables > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:d}", "Number of variables:",
                env->problem->properties.numberOfVariables, env->reformulatedProblem->properties.numberOfVariables));

        if(env->problem->properties.numberOfRealVariables > 0
            || env->reformulatedProblem->properties.numberOfRealVariables > 0)
            env->output->outputInfo(
                fmt::format(" {:35s}{:<21d}{:d}", " - real:", env->problem->properties.numberOfRealVariables,
                    env->reformulatedProblem->properties.numberOfRealVariables));

        if(env->problem->properties.numberOfBinaryVariables > 0
            || env->reformulatedProblem->properties.numberOfBinaryVariables > 0)
            env->output->outputInfo(
                fmt::format(" {:35s}{:<21d}{:d}", " - binary:", env->problem->properties.numberOfBinaryVariables,
                    env->reformulatedProblem->properties.numberOfBinaryVariables));

        if(env->problem->properties.numberOfIntegerVariables > 0
            || env->reformulatedProblem->properties.numberOfIntegerVariables > 0)
            env->output->outputInfo(
                fmt::format(" {:35s}{:<21d}{:d}", " - integer:", env->problem->properties.numberOfIntegerVariables,
                    env->reformulatedProblem->properties.numberOfIntegerVariables));

        if(env->problem->properties.numberOfSemicontinuousVariables > 0
            || env->reformulatedProblem->properties.numberOfSemicontinuousVariables > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:d}",
                " - semicontinuous:", env->problem->properties.numberOfSemicontinuousVariables,
                env->reformulatedProblem->properties.numberOfSemicontinuousVariables));

        if(env->problem->properties.numberOfSemiintegerVariables > 0
            || env->reformulatedProblem->properties.numberOfSemiintegerVariables > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:d}",
                " - semiinteger:", env->problem->properties.numberOfSemiintegerVariables,
                env->reformulatedProblem->properties.numberOfSemiintegerVariables));

        if(env->problem->properties.numberOfNonlinearVariables > 0
            || env->reformulatedProblem->properties.numberOfNonlinearVariables > 0)
            env->output->outputInfo(
                fmt::format(" {:35s}{:<21d}{:d}", " - nonlinear:", env->problem->properties.numberOfNonlinearVariables,
                    env->reformulatedProblem->properties.numberOfNonlinearVariables));
    }
    else
    {
        if(env->problem->properties.numberOfVariables > 0)
            env->output->outputInfo(fmt::format(
                " {:35s}{:<21d}{:s}", "Number of variables:", env->problem->properties.numberOfVariables, ""));

        if(env->problem->properties.numberOfRealVariables > 0)
            env->output->outputInfo(
                fmt::format(" {:35s}{:<21d}{:s}", " - real:", env->problem->properties.numberOfRealVariables, ""));

        if(env->problem->properties.numberOfBinaryVariables > 0)
            env->output->outputInfo(
                fmt::format(" {:35s}{:<21d}{:s}", " - binary:", env->problem->properties.numberOfBinaryVariables, ""));

        if(env->problem->properties.numberOfIntegerVariables > 0)
            env->output->outputInfo(fmt::format(
                " {:35s}{:<21d}{:s}", " - integer:", env->problem->properties.numberOfIntegerVariables, ""));

        if(env->problem->properties.numberOfSemicontinuousVariables > 0)
            env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:s}",
                " - semicontinuous:", env->problem->properties.numberOfSemicontinuousVariables, ""));

        if(env->problem->properties.numberOfSemiintegerVariables > 0)
            env->output->outputInfo(fmt::format(
                " {:35s}{:<21d}{:s}", " - semiinteger:", env->problem->properties.numberOfSemiintegerVariables, ""));
    }

    if(env->problem->properties.numberOfSpecialOrderedSets
            + env->reformulatedProblem->properties.numberOfSpecialOrderedSets
        > 0)
    {
        env->output->outputInfo("");

        if(isReformulated)
        {
            if(env->problem->properties.numberOfVariables > 0
                || env->reformulatedProblem->properties.numberOfSpecialOrderedSets > 0)
                env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:d}",
                    "Number of special ordered sets:", env->problem->properties.numberOfSpecialOrderedSets,
                    env->reformulatedProblem->properties.numberOfSpecialOrderedSets));
        }
        else
        {
            if(env->problem->properties.numberOfVariables > 0)
                env->output->outputInfo(fmt::format(" {:35s}{:<21d}{:s}",
                    "Number of special ordered sets:", env->problem->properties.numberOfSpecialOrderedSets, ""));
        }
    }

    if(env->results->auxiliaryVariablesIntroduced.size() > 0 || env->reformulatedProblem->antiEpigraphObjectiveVariable)
    {
        int totalNumberOfTransformations = 0;

        for(auto& AUXVAR : env->results->auxiliaryVariablesIntroduced)
            totalNumberOfTransformations += AUXVAR.second;

        if(env->reformulatedProblem->antiEpigraphObjectiveVariable)
            totalNumberOfTransformations++;

        env->output->outputInfo("");

        env->output->outputInfo(
            fmt::format(" {:56s}{:d}", "Number of transformations performed:", totalNumberOfTransformations));

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::NonlinearObjectiveFunction);
            value > 0)
            env->output->outputInfo(fmt::format(" {:56s}{:d}", " - epigraph:", "", value));

        if(auto value
            = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::NonlinearExpressionPartitioning);
            value > 0)
            env->output->outputInfo(fmt::format(" {:56s}{:d}", " - nonlinear expression partitioning:", value));

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::MonomialTermsPartitioning);
            value > 0)
            env->output->outputInfo(fmt::format(" {:56s}{:d}", " - monomial terms partitioning:", value));

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::SignomialTermsPartitioning);
            value > 0)
            env->output->outputInfo(fmt::format(" {:56s}{:d}", " - signomial terms partitioning:", value));

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::SquareTermsPartitioning);
            value > 0)
            env->output->outputInfo(fmt::format(" {:56s}{:d}", " - square terms partitioning:", value));

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::ContinuousBilinear);
            value > 0)
            env->output->outputInfo(fmt::format(" {:56s}{:d}", " - continuous bilinear term extraction:", value));

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::BinaryBilinear); value > 0)
            env->output->outputInfo(fmt::format(" {:56s}{:d}", " - binary bilinear term reformulation:", value));

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::BinaryContinuousBilinear);
            value > 0)
            env->output->outputInfo(
                fmt::format(" {:56s}{:d}", " - binary/continuous bilinear term reformulation:", value));

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::IntegerBilinear); value > 0)
            env->output->outputInfo(fmt::format(" {:56s}{:d}", " - integer bilinear term reformulation:", value));

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::BinaryMonomial); value > 0)
            env->output->outputInfo(fmt::format(" {:56s}{:d}", " - binary monomial term reformulation:", value));

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::AbsoluteValue); value > 0)
            env->output->outputInfo(fmt::format(" {:56s}{:d}", " - absolute value reformulation:", value));

        if(env->reformulatedProblem->antiEpigraphObjectiveVariable)
            env->output->outputInfo(fmt::format(" {:56s}", " - anti-epigraph reformulation"));

        if(auto value = env->results->getAuxiliaryVariableCounter(E_AuxiliaryVariableType::EigenvalueDecomposition);
            value > 0)
            env->output->outputInfo(fmt::format(" {:56s}{:d}", " - quadratic eigenvalue decomposition:", value));
    }
}

void Report::outputSolutionReport()
{
    std::stringstream report;

    env->output->outputInfo("");

#ifdef SIMPLE_OUTPUT_CHARS
    env->output->outputInfo(
        "- Solution report "
        "-----------------------------------------------------------------------------------------------------");
#else
    env->output->outputInfo(
        "╶ Solution report "
        "────────────────────────────────────────────────────────────────────────────────────────────────────╴");
#endif

    env->output->outputInfo("");

    bool primalSolutionFound = env->results->hasPrimalSolution();

    if(env->results->terminationReasonDescription != "")
        env->output->outputInfo(fmt::format(" {}", env->results->terminationReasonDescription));

    env->output->outputInfo("");

    switch(env->results->getModelReturnStatus())
    {
    case E_ModelReturnStatus::OptimalGlobal:
        env->output->outputInfo(" Globally optimal primal solution found.");
        break;
    case E_ModelReturnStatus::FeasibleSolution:
        if(env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Convex)
            env->output->outputInfo(
                " Feasible primal solution found to convex problem. Can not guarantee optimality to the given "
                "termination criteria.");
        else
            env->output->outputInfo(
                " Feasible primal solution found to nonconvex problem. Can not guarantee optimality to the given "
                "termination criteria.");
        break;
    case E_ModelReturnStatus::InfeasibleLocal:
        env->output->outputInfo(
            " Problem found to be infeasible, but globality could not be verified since the problem seems to be "
            "nonconvex.");
        break;
    case E_ModelReturnStatus::InfeasibleGlobal:
        env->output->outputInfo(" Problem is infeasible.");
        break;
    case E_ModelReturnStatus::Unbounded:
        env->output->outputInfo(" Problem is unbounded, but a primal solution was found.");
        break;
    case E_ModelReturnStatus::UnboundedNoSolution:
        env->output->outputInfo(" Problem is unbounded, and no primal solution was found.");
        break;
    case E_ModelReturnStatus::NoSolutionReturned:
        env->output->outputInfo(" No solution found. Try modifying the termination criteria.");
        break;
    case E_ModelReturnStatus::ErrorUnknown:
        env->output->outputInfo(" An error occurred, but a primal solution was found.");
        break;
    case E_ModelReturnStatus::None:
    case E_ModelReturnStatus::ErrorNoSolution:
        env->output->outputInfo(" An error occurred, and no primal solution was found.");
    };

    env->output->outputInfo("");

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
        {
            env->output->outputInfo(
                " Warning! Solution point is at maximum variable bounds. Problem might be artificially bounded.");
            env->output->outputInfo("");
        }
    }

    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        env->output->outputInfo(fmt::format(" Objective bound (minimization) [dual, primal]:  [{:g}, {:g}].",
            env->results->getGlobalDualBound(), env->results->getPrimalBound()));
    }
    else
    {
        env->output->outputInfo(fmt::format(" Objective bound (maximization) [primal, dual]:  [{:g}, {:g}].",
            env->results->getPrimalBound(), env->results->getGlobalDualBound()));
    }

    env->output->outputInfo(fmt::format(" Objective gap absolute / relative:              {:g} / {:g}.",
        env->results->getAbsoluteGlobalObjectiveGap(), env->results->getRelativeGlobalObjectiveGap()));

    env->output->outputInfo("");

    std::vector<std::string> fulfilled;
    std::vector<std::string> unfulfilled;

    if(env->results->isAbsoluteObjectiveGapToleranceMet())
    {
        fulfilled.push_back(fmt::format("  - absolute objective gap tolerance             {:g} <= {:g}",
            env->results->getAbsoluteGlobalObjectiveGap(),
            env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination")));
    }
    else
    {
        unfulfilled.push_back(fmt::format("  - absolute objective gap tolerance             {:g} > {:g}",
            env->results->getAbsoluteGlobalObjectiveGap(),
            env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination")));
    }

    if(env->results->isRelativeObjectiveGapToleranceMet())
    {
        fulfilled.push_back(fmt::format("  - relative objective gap tolerance             {:g} <= {:g}",
            env->results->getRelativeGlobalObjectiveGap(),
            env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination")));
    }
    else
    {
        unfulfilled.push_back(fmt::format("  - relative objective gap tolerance             {:g} > {:g}",
            env->results->getRelativeGlobalObjectiveGap(),
            env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination")));
    }

    if(static_cast<ES_TreeStrategy>(env->settings->getSetting<int>("TreeStrategy", "Dual"))
        != ES_TreeStrategy::SingleTree)
    {
        if(env->results->getCurrentIteration()->maxDeviation
            <= env->settings->getSetting<double>("ConstraintTolerance", "Termination"))
        {
            fulfilled.push_back(fmt::format("  - maximal constraint tolerance                 {:g} <= {:g}",
                env->results->getCurrentIteration()->maxDeviation,
                env->settings->getSetting<double>("ConstraintTolerance", "Termination")));
        }
        else
        {
            unfulfilled.push_back(fmt::format("  - maximal constraint tolerance                 {:g} > {:g}",
                env->results->getCurrentIteration()->maxDeviation,
                env->settings->getSetting<double>("ConstraintTolerance", "Termination")));
        }
    }

    ;

    if(int iterLim = env->settings->getSetting<int>("IterationLimit", "Termination");
        env->results->getCurrentIteration()->iterationNumber > iterLim)
    {
        fulfilled.push_back(fmt::format("  - iteration limit                              {} > {}",
            env->results->getCurrentIteration()->iterationNumber, iterLim));
    }
    else
    {
        unfulfilled.push_back(fmt::format("  - iteration limit                              {} <= {}",
            env->results->getCurrentIteration()->iterationNumber, iterLim));
    }

    auto timeLimit = env->settings->getSetting<double>("TimeLimit", "Termination");

    if(auto totalTime = env->timing->getElapsedTime("Total"); totalTime > timeLimit)
    {
        fulfilled.push_back(
            fmt::format("  - solution time limit (s)                      {:g} > {:g}", totalTime, timeLimit));
    }
    else
    {
        unfulfilled.push_back(
            fmt::format("  - solution time limit (s)                      {:g} <= {:g}", totalTime, timeLimit));
    }

    env->output->outputInfo(" Fulfilled termination criteria: ");

    for(auto const& COND : fulfilled)
        env->output->outputInfo(COND);

    env->output->outputInfo("");

    env->output->outputInfo(" Unfulfilled termination criteria:");

    for(auto const& COND : unfulfilled)
        env->output->outputInfo(COND);

    env->output->outputInfo("");

    env->output->outputInfo(fmt::format(
        " Dual problems solved in main step:              {}", env->solutionStatistics.getNumberOfTotalDualProblems()));

    if(env->solutionStatistics.numberOfProblemsLP > 0)
    {
        env->output->outputInfo(fmt::format(
            "  - LP problems                                  {}", env->solutionStatistics.numberOfProblemsLP));
    }

    if(env->solutionStatistics.numberOfProblemsQP > 0)
    {
        env->output->outputInfo(fmt::format(
            "  - QP problems                                  {}", env->solutionStatistics.numberOfProblemsQP));
    }

    if(env->solutionStatistics.numberOfProblemsQCQP > 0)
    {
        env->output->outputInfo(fmt::format(
            "  - QCQP problems                                {}", env->solutionStatistics.numberOfProblemsQCQP));
    }

    if(env->solutionStatistics.numberOfProblemsOptimalMILP > 0)
    {
        env->output->outputInfo(fmt::format("  - MILP problems, optimal                       {}",
            env->solutionStatistics.numberOfProblemsOptimalMILP));
    }

    if(env->solutionStatistics.numberOfProblemsFeasibleMILP > 0)
    {
        env->output->outputInfo(fmt::format("  - MILP problems, feasible                      {}",
            env->solutionStatistics.numberOfProblemsFeasibleMILP));
    }

    if(env->solutionStatistics.numberOfProblemsOptimalMIQP > 0)
    {
        env->output->outputInfo(fmt::format("  - MIQP problems, optimal                       {}",
            env->solutionStatistics.numberOfProblemsOptimalMIQP));
    }

    if(env->solutionStatistics.numberOfProblemsFeasibleMIQP > 0)
    {
        env->output->outputInfo(fmt::format("  - MIQP problems, feasible                      {}",
            env->solutionStatistics.numberOfProblemsFeasibleMIQP));
    }

    env->output->outputInfo("");

    if(env->solutionStatistics.numberOfExploredNodes > 0)
    {
        env->output->outputInfo(fmt::format(
            " Number of explored nodes:                       {}", env->solutionStatistics.numberOfExploredNodes));
        env->output->outputInfo("");
    }

    if(env->solutionStatistics.numberOfProblemsMinimaxLP > 0)
    {
        env->output->outputInfo(" Problems solved during interior point search:");
        env->output->outputInfo(fmt::format(
            " - LP problems:                                  {}", env->solutionStatistics.numberOfProblemsMinimaxLP));
        env->output->outputInfo("");
    }

    if(env->solutionStatistics.numberOfProblemsFixedNLP > 0)
    {
        env->output->outputInfo(fmt::format(
            " Fixed primal NLP problems solved:               {}", env->solutionStatistics.numberOfProblemsFixedNLP));
        env->output->outputInfo("");
    }

    if(env->results->hasPrimalSolution())
    {
        env->output->outputInfo(fmt::format(
            " {:<48}{:d}", "Number of primal solutions found:", env->solutionStatistics.numberOfFoundPrimalSolutions));

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
            case E_PrimalSolutionSource::MIPCallback:
                sourceDesc = "MIP callback";
                break;
            case E_PrimalSolutionSource::InteriorPointSearch:
                sourceDesc = "Interior point search";
                break;
            default:
                sourceDesc = "other";
                break;
            }

            env->output->outputInfo(fmt::format(" - {:<46}{:d}", sourceDesc + ':', S.second));
        }

        env->output->outputInfo("");
    }

    for(auto& T : env->timing->timers)
    {
        T.stop();
        auto elapsed = T.elapsed();

        if(elapsed > 0)
            env->output->outputInfo(fmt::format(" {:<48}{:g}", T.description + ':', elapsed));
    }
}

void Report::outputInteriorPointPreReport()
{
    std::stringstream report;

    env->output->outputInfo("");

#ifdef SIMPLE_OUTPUT_CHARS
    env->output->outputInfo(
        "- Interior point search "
        "-----------------------------------------------------------------------------------------------");
#else
    env->output->outputInfo(
        "╶ Interior point search "
        "──────────────────────────────────────────────────────────────────────────────────────────────╴");
#endif

    env->output->outputInfo("");
    env->output->outputInfo(" Strategy selected:          cutting plane minimax");

    env->output->outputInfo(report.str());
}

void Report::outputPreReport()
{
    std::stringstream report;

    env->output->outputInfo("");

#ifdef SIMPLE_OUTPUT_CHARS
    env->output->outputInfo(
        "- Main iteration step "
        "-------------------------------------------------------------------------------------------------");
#else
    env->output->outputInfo(
        "╶ Main iteration step "
        "────────────────────────────────────────────────────────────────────────────────────────────────╴");
#endif

    env->output->outputInfo(report.str());
}
} // namespace SHOT