/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Report.h"

#include "Iteration.h"
#include "Output.h"
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
            firstIterationHeaderPrinted = true;
        }

        if(iterationPrintoutsSinceLastHeader > 75)
        {
            this->outputIterationDetailHeader();
        }

        iterationsWithoutPrintoutCounter = 0;
        iterationPrintoutsSinceLastHeader++;
        lastIterationOutputTimeStamp = env->timing->getElapsedTime("Total");

        std::string combDualCuts = "";

        if(dualCutsAdded > 0)
        {
            combDualCuts = fmt::format("{:>4d} | {:<6d}", dualCutsAdded, dualCutsTotal);
        }

        std::string combObjectiveValue;

        if(lineType == E_IterationLineType::DualReductionCut)
        {
            combObjectiveValue
                = fmt::format("Cutoff: {:<12s}", Utilities::toStringFormat(primalObjectiveValue, "{:g}"));
        }
        else
        {
            if(env->problem->objectiveFunction->properties.isMinimize)
            {
                combObjectiveValue = fmt::format("{:>12s}{}| {:<12s}",
                    Utilities::toStringFormat(dualObjectiveValue, "{:g}"), env->results->solutionIsGlobal ? " " : "*",
                    Utilities::toStringFormat(primalObjectiveValue, "{:g}"));
            }
            else
            {
                combObjectiveValue = fmt::format("{:>12s} | {:<12s}{}",
                    Utilities::toStringFormat(primalObjectiveValue, "{:g}"),
                    Utilities::toStringFormat(dualObjectiveValue, "{:g}"), env->results->solutionIsGlobal ? " " : "*");
            }
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
            combCurrSol = fmt::format("{:>12.2g} | {}", currentObjectiveValue, 0.0);
        }
        else
        {
            combCurrSol
                = fmt::format("{:>12g} | {}: {:.2e}", currentObjectiveValue, maxConstraintIndex, maxConstraintError);
        }

        if(lineType == E_IterationLineType::DualRepair)
        {
            auto tmpLine = fmt::format("{:>6d}: {:<10s}{:^10.2f}{:>13s}{:>27s}{:>19s}{:<32s}", iterationNumber,
                iterationDesc, totalTime, combDualCuts, "", "", "");
            env->output->outputInfo(tmpLine);
        }
        else if(lineType == E_IterationLineType::DualReductionCut)
        {
            auto tmpLine = fmt::format("{:>6d}: {:<10s}{:^10.2f}{:>13s}{:>27s}{:>19s}{:<32s}", iterationNumber,
                iterationDesc, totalTime, combDualCuts, combObjectiveValue, "", "");
            env->output->outputInfo(tmpLine);
        }
        else
        {
            auto tmpLine = fmt::format("{:>6d}: {:<10s}{:^10.2f}{:>13s}{:>27s}{:>19s}{:<32s}", iterationNumber,
                iterationDesc, totalTime, combDualCuts, combObjectiveValue, combObjectiveGap, combCurrSol);
            env->output->outputInfo(tmpLine);
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

        nodes << "\r\n";

        env->output->outputDebug(nodes.str());
    }
    catch(...)
    {
        env->output->outputError("Cannot write iteration solution report!");
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
    std::stringstream header;

    header << "\r\n";
    header << "                                                                                     \n";

    header << "    Iteration     │  Time  │  Dual cuts  │     Objective value     │   Objective gap   │     Current "
              "solution\r\n";

    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        header
            << "     #: type      │  tot.  │   + | tot.  │       dual | primal     │    abs. | rel.    │    obj.fn. | "
               "max.err.\r\n";
    }
    else
    {
        header
            << "     #: type      │  tot.  │   + | tot.  │     primal | dual       │    abs. | rel.    │    obj.fn. | "
               "max.err.\r\n";
    }
    header << "╶─────────────────┴────────┴─────────────┴─────────────────────────┴───────────────────┴────────────"
              "────"
              "───────────╴\r\n";
    header << "\r\n";

    env->output->outputInfo(header.str());
    iterationPrintoutsSinceLastHeader = 0;
}

void Report::outputIterationDetailHeaderMinimax()
{
    std::stringstream header;
    header << "                                                                                     \n";

    header << "    Iteration     │  Time  │    Cuts     │     Objective value     │  Objective diff.   \r\n";
    header << "     #: type      │  tot.  │   + | tot.  │    problem | line srch  │    abs. | rel.    \r\n";
    header << "╶─────────────────┴────────┴─────────────┴─────────────────────────┴──────────────────╴\r\n";

    env->output->outputInfo(header.str());
}

void Report::outputSolverHeader()
{
    std::stringstream header;

    header << "\r\n";
    header << "╶ Supporting Hyperplane Optimization Toolkit (SHOT) "
              "──────────────────────────────────────────────────────────────────╴\r\n";
    header << "\r\n";

    header << "  Andreas Lundell and Jan Kronqvist, Åbo Akademi University, Finland.\r\n";
    header << "  See documentation for full list of contributors and utilized software libraries.\r\n";

    header << "\r\n";
    header << "  Version: ";
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

    header << "\r\n";
    header << "\r\n";
    header << "  For more information visit https://shotsolver.dev\r\n";

    env->output->outputInfo(header.str());
}

void Report::outputOptionsReport()
{
    std::stringstream report;

    report << "\r\n";
    report << "╶ Options "
              "────────────────────────────────────────────────────────────────────────────────────────────────────────"
              "────╴\r\n";
    report << "\r\n";

    auto optionsFile = env->settings->getSetting<std::string>("OptionsFile", "Input");

    if(optionsFile == "")
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

    auto nonDefaultSettings = env->settings->getChangedSettings();

    if(nonDefaultSettings.size() > 0)
    {
        report << " Nondefault options used:\r\n";
        report << "\r\n";

        for(auto& S : nonDefaultSettings)
        {
            report << "  - " << S << "\r\n";
        }

        report << "\r\n";
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
            if(env->settings->getSetting<bool>("Cplex.UseNewCallbackType", "Subsolver"))
            {
                dualSolver = "CPLEX with new callback functionality";
            }
            else
            {
                dualSolver = "CPLEX with old callback functionality";
            }
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
        report << " Dual strategy:              Single-tree\r\n";
        report << "  - cut algorithm:           " << cutAlgorithm << "\r\n";
        break;
    case(E_SolutionStrategy::MultiTree):
        report << " Dual strategy:              Multi-tree\r\n";
        report << "  - cut algorithm:           " << cutAlgorithm << "\r\n";
        break;
    case(E_SolutionStrategy::NLP):
        report << " Dual strategy:              NLP version\r\n";
        report << "  - cut algorithm:           " << cutAlgorithm << "\r\n";
        break;
    case(E_SolutionStrategy::MIQP):
        report << " Dual strategy:              MIQP version\r\n";
        break;
    case(E_SolutionStrategy::MIQCQP):
        report << " Dual strategy:              MIQCQP version\r\n";
        break;
    default:
        break;
    }

    report << "  - solver:                  " << dualSolver << "\r\n";

    report << "\r\n";

    report << " Primal NLP solver:          ";

    switch(static_cast<ES_PrimalNLPSolver>(env->results->usedPrimalNLPSolver))
    {
    case(ES_PrimalNLPSolver::None):
        report << "none";
        break;
    case(ES_PrimalNLPSolver::GAMS):
        report << "GAMS (";
        report << env->settings->getSetting<std::string>("GAMS.NLP.Solver", "Subsolver");
        report << ")\r\n";
        break;
    case(ES_PrimalNLPSolver::Ipopt):
        report << "Ipopt ";

        switch(static_cast<ES_IpoptSolver>(env->settings->getSetting<int>("Ipopt.LinearSolver", "Subsolver")))
        {
        case(ES_IpoptSolver::ma27):
            report << "with HSL MA27 linear solver";
            break;

        case(ES_IpoptSolver::ma57):
            report << "with HSL MA57 linear solver";
            break;

        case(ES_IpoptSolver::ma86):
            report << "with HSL MA86 linear solver";
            break;

        case(ES_IpoptSolver::ma97):
            report << "with HSL MA97 linear solver";
            break;

        case(ES_IpoptSolver::mumps):
            report << "with MUMPS linear solver";
            break;
        default:
            report << "with Ipopt default linear solver";
        }

        report << "\r\n";
        break;
    default:
        report << "none";
        break;
    }

    report << "\r\n";

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        report << " Debug directory:            ";
        report << env->settings->getSetting<std::string>("Debug.Path", "Output");
        report << "\r\n";
    }

    env->output->outputInfo(report.str());
}

void Report::outputProblemInstanceReport()
{
    std::stringstream report;

    bool isReformulated = (env->problem == env->reformulatedProblem) ? false : true;

    report << "╶ Problem instance "
              "───────────────────────────────────────────────────────────────────────────────────────────────────╴"
              "\r\n";
    report << "\r\n";

    auto problemFile = env->settings->getSetting<std::string>("ProblemFile", "Input");

    report << " Problem read from file:     " << problemFile;
    report << "\r\n\r\n";

    if(isReformulated)
    {
        report << fmt::format(" {:28s}{:21s}{:s}", "", "Original", "Reformulated") << "\r\n";
    }
    else
    {
        report << fmt::format(" {:28s}{:21s}{:s}", "", "Original", "") << "\r\n";
    }

    report << "\r\n";

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
                  " {:28s}{:21s}{:s}", "Problem classification:", problemClassificationOrig, problemClassificationRef)
           << "\r\n";

    std::string objectiveClassificationOrig;
    std::string objectiveClassificationRef = "";

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

    if(isReformulated)
    {
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
    }

    report << fmt::format(" {:28s}{:21s}{:s}", "Objective function type:", objectiveClassificationOrig,
                  objectiveClassificationRef)
           << "\r\n";

    report << "\r\n";

    if(isReformulated)
    {
        if(env->problem->properties.numberOfNumericConstraints > 0
            || env->reformulatedProblem->properties.numberOfNumericConstraints > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}",
                          "Number of constraints:", env->problem->properties.numberOfNumericConstraints,
                          env->reformulatedProblem->properties.numberOfNumericConstraints)
                   << "\r\n";

        if(env->problem->properties.numberOfLinearConstraints > 0
            || env->reformulatedProblem->properties.numberOfLinearConstraints > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}",
                          " - linear:", env->problem->properties.numberOfLinearConstraints,
                          env->reformulatedProblem->properties.numberOfLinearConstraints)

                   << "\r\n";

        if(env->problem->properties.numberOfQuadraticConstraints > 0
            || env->reformulatedProblem->properties.numberOfQuadraticConstraints > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}",
                          " - quadratic:", env->problem->properties.numberOfQuadraticConstraints,
                          env->reformulatedProblem->properties.numberOfQuadraticConstraints)

                   << "\r\n";

        if(env->problem->properties.numberOfNonlinearConstraints > 0
            || env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}",
                          " - nonlinear:", env->problem->properties.numberOfNonlinearConstraints,
                          env->reformulatedProblem->properties.numberOfNonlinearConstraints)
                   << "\r\n";
    }
    else
    {
        if(env->problem->properties.numberOfNumericConstraints > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}",
                          "Number of constraints:", env->problem->properties.numberOfNumericConstraints, "")
                   << "\r\n";

        if(env->problem->properties.numberOfLinearConstraints > 0)
            report << fmt::format(
                          " {:28s}{:<21d}{:d}", " - linear:", env->problem->properties.numberOfLinearConstraints, "")
                   << "\r\n";

        if(env->problem->properties.numberOfQuadraticConstraints > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}",
                          " - quadratic:", env->problem->properties.numberOfQuadraticConstraints, "")
                   << "\r\n";

        if(env->problem->properties.numberOfNonlinearConstraints > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}",
                          " - nonlinear:", env->problem->properties.numberOfNonlinearConstraints, "")
                   << "\r\n";
    }

    report << "\r\n";

    if(isReformulated)
    {
        if(env->problem->properties.numberOfVariables > 0 || env->reformulatedProblem->properties.numberOfVariables > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}",
                          "Number of variables:", env->problem->properties.numberOfVariables,
                          env->reformulatedProblem->properties.numberOfVariables)
                   << "\r\n";

        if(env->problem->properties.numberOfRealVariables > 0
            || env->reformulatedProblem->properties.numberOfRealVariables > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}", " - real:", env->problem->properties.numberOfRealVariables,
                          env->reformulatedProblem->properties.numberOfRealVariables)
                   << "\r\n";

        if(env->problem->properties.numberOfBinaryVariables > 0
            || env->reformulatedProblem->properties.numberOfBinaryVariables > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}", " - binary:", env->problem->properties.numberOfBinaryVariables,
                          env->reformulatedProblem->properties.numberOfBinaryVariables)
                   << "\r\n";

        if(env->problem->properties.numberOfIntegerVariables > 0
            || env->reformulatedProblem->properties.numberOfIntegerVariables > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}",
                          " - integer:", env->problem->properties.numberOfIntegerVariables,
                          env->reformulatedProblem->properties.numberOfIntegerVariables)
                   << "\r\n";

        if(env->problem->properties.numberOfSemicontinuousVariables > 0
            || env->reformulatedProblem->properties.numberOfSemicontinuousVariables > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}",
                          " - semicontinuous:", env->problem->properties.numberOfSemicontinuousVariables,
                          env->reformulatedProblem->properties.numberOfSemicontinuousVariables)
                   << "\r\n";
    }
    else
    {
        if(env->problem->properties.numberOfVariables > 0)
            report << fmt::format(
                          " {:28s}{:<21d}{:d}", "Number of variables:", env->problem->properties.numberOfVariables, "")
                   << "\r\n";

        if(env->problem->properties.numberOfRealVariables > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}", " - real:", env->problem->properties.numberOfRealVariables, "")
                   << "\r\n";

        if(env->problem->properties.numberOfBinaryVariables > 0)
            report << fmt::format(
                          " {:28s}{:<21d}{:d}", " - binary:", env->problem->properties.numberOfBinaryVariables, "")
                   << "\r\n";

        if(env->problem->properties.numberOfIntegerVariables > 0)
            report << fmt::format(
                          " {:28s}{:<21d}{:d}", " - integer:", env->problem->properties.numberOfIntegerVariables, "")
                   << "\r\n";

        if(env->problem->properties.numberOfSemicontinuousVariables > 0)
            report << fmt::format(" {:28s}{:<21d}{:d}",
                          " - semicontinuous:", env->problem->properties.numberOfSemicontinuousVariables, "")
                   << "\r\n";
    }

    env->output->outputInfo(report.str());
}

void Report::outputSolutionReport()
{
    std::stringstream report;

    report << "\r\n\r\n";
    report << "╶ Solution report "
              "────────────────────────────────────────────────────────────────────────────────────────────────────"
              "╴\r\n";
    report << "\r\n";

    bool primalSolutionFound = env->results->hasPrimalSolution();

    if(env->results->terminationReasonDescription != "")
        report << " " << env->results->terminationReasonDescription << "\r\n\r\n";

    switch(env->results->getModelReturnStatus())
    {
    /* case E_ModelReturnStatus::OptimalLocal:
        report << " Optimal primal solution found, but it might be local since the model seems to be nonconvex.\r\n";
        break;*/
    case E_ModelReturnStatus::OptimalGlobal:
        report << " Globally optimal primal solution found.\r\n";
        break;
    case E_ModelReturnStatus::FeasibleSolution:
        report << " Feasible primal solution found. Can not guarantee optimality to the given termination "
                  "criteria.\r\n";
        break;
    case E_ModelReturnStatus::InfeasibleLocal:
        report << " Problem found to be infeasible, but globality could not be verified since the problem seems to be "
                  "nonconvex.\r\n";
        break;
    case E_ModelReturnStatus::InfeasibleGlobal:
        report << " Problem is infeasible.\r\n";
        break;
    case E_ModelReturnStatus::Unbounded:
        report << " Problem is unbounded, but a primal solution was found.\r\n";
        break;
    case E_ModelReturnStatus::UnboundedNoSolution:
        report << " Problem is unbounded, and no primal solution was found.\r\n";
        break;
    case E_ModelReturnStatus::NoSolutionReturned:
        report << " No solution found. Try modifying the termination criteria.\r\n";
        break;
    case E_ModelReturnStatus::ErrorUnknown:
        report << " An error occurred, but a primal solution was found.\r\n";
        break;
    case E_ModelReturnStatus::None:
    case E_ModelReturnStatus::ErrorNoSolution:
        report << " An error occurred, and no primal solution was found.\r\n";
    };

    // Warn the user if variables are at maximum bound limits

    if(primalSolutionFound)
    {
        bool variablesAreBounded = true;
        double minLBCont = env->settings->getSetting<double>("ContinuousVariable.MinimumLowerBound", "Model");
        double maxUBCont = env->settings->getSetting<double>("ContinuousVariable.MaximumUpperBound", "Model");
        double minLBInt = env->settings->getSetting<double>("IntegerVariable.MinimumLowerBound", "Model");
        double maxUBInt = env->settings->getSetting<double>("IntegerVariable.MaximumUpperBound", "Model");

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
                << " Warning! Solution point is at maximum variable bounds. Problem might be artificially bounded.\r\n";
    }

    report << "\r\n";

    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        report << " Objective bound (minimization) [dual, primal]:  ";
        report << "[" << Utilities::toStringFormat(env->results->getGlobalDualBound(), "{:g}") << ", ";
        report << Utilities::toStringFormat(env->results->getPrimalBound(), "{:g}") << "]\r\n";
    }
    else
    {
        report << " Objective bound (maximization) [primal, dual]:  ";
        report << "[" << Utilities::toStringFormat(env->results->getPrimalBound(), "{:g}") << ", ";
        report << Utilities::toStringFormat(env->results->getGlobalDualBound(), "{:g}") << "]\r\n";
    }

    report << " Objective gap absolute / relative:              ";
    report << "" << Utilities::toStringFormat(env->results->getAbsoluteGlobalObjectiveGap(), "{:g}") << " / ";
    report << Utilities::toStringFormat(env->results->getRelativeGlobalObjectiveGap(), "{:g}") << "\r\n";
    report << "\r\n";

    std::stringstream fulfilled;
    std::stringstream unfulfilled;

    if(env->results->isAbsoluteObjectiveGapToleranceMet())
    {
        fulfilled << "  - absolute objective gap tolerance             ";
        fulfilled << env->results->getAbsoluteGlobalObjectiveGap() << " <= ";
        fulfilled << env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") << "\r\n";
    }
    else
    {
        unfulfilled << "  - absolute objective gap tolerance             ";
        unfulfilled << env->results->getAbsoluteGlobalObjectiveGap() << " > ";
        unfulfilled << env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") << "\r\n";
    }

    if(env->results->isRelativeObjectiveGapToleranceMet())
    {
        fulfilled << "  - relative objective gap tolerance             ";
        fulfilled << env->results->getRelativeGlobalObjectiveGap() << " <= ";
        fulfilled << env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination") << "\r\n";
    }
    else
    {
        unfulfilled << "  - relative objective gap tolerance             ";
        unfulfilled << env->results->getRelativeGlobalObjectiveGap() << " > ";
        unfulfilled << env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination") << "\r\n";
    }

    if(static_cast<ES_TreeStrategy>(env->settings->getSetting<int>("TreeStrategy", "Dual"))
        != ES_TreeStrategy::SingleTree)
    {
        if(env->results->getCurrentIteration()->maxDeviation
            <= env->settings->getSetting<double>("ConstraintTolerance", "Termination"))
        {
            fulfilled << "  - maximal constraint tolerance                 ";
            fulfilled << env->results->getCurrentIteration()->maxDeviation << " <= ";
            fulfilled << env->settings->getSetting<double>("ConstraintTolerance", "Termination") << "\r\n";
        }
        else
        {
            unfulfilled << "  - maximal constraint tolerance                 ";
            unfulfilled << env->results->getCurrentIteration()->maxDeviation << " > ";
            unfulfilled << env->settings->getSetting<double>("ConstraintTolerance", "Termination") << "\r\n";
        }
    }

    int iterLim = env->settings->getSetting<int>("IterationLimit", "Termination");

    if(env->results->getCurrentIteration()->iterationNumber > iterLim)
    {
        fulfilled << "  - iteration limit                              ";
        fulfilled << env->results->getCurrentIteration()->iterationNumber << " > ";
        fulfilled << iterLim << "\r\n";
    }
    else
    {
        unfulfilled << "  - iteration limit                              ";
        unfulfilled << env->results->getCurrentIteration()->iterationNumber << " <= ";
        unfulfilled << iterLim << "\r\n";
    }

    if(env->timing->getElapsedTime("Total") > env->settings->getSetting<double>("TimeLimit", "Termination"))
    {
        fulfilled << "  - solution time limit (s)                      ";
        fulfilled << env->timing->getElapsedTime("Total") << " > ";
        fulfilled << env->settings->getSetting<double>("TimeLimit", "Termination") << "\r\n";
    }
    else
    {
        unfulfilled << "  - solution time limit (s)                      ";
        unfulfilled << env->timing->getElapsedTime("Total") << " <= ";
        unfulfilled << env->settings->getSetting<double>("TimeLimit", "Termination") << "\r\n";
    }

    report << " Fulfilled termination criteria: \r\n";
    report << fulfilled.str();
    report << "\r\n";

    report << " Unfulfilled termination criteria: \r\n";
    report << unfulfilled.str();
    report << "\r\n";

    report << " Dual problems solved in main step:              ";
    report << env->solutionStatistics.getNumberOfTotalDualProblems() << "\r\n";

    if(env->solutionStatistics.numberOfProblemsLP > 0)
    {
        report << "  - LP problems                                  " << env->solutionStatistics.numberOfProblemsLP
               << "\r\n";
    }

    if(env->solutionStatistics.numberOfProblemsQP > 0)
    {
        report << "  - QP problems                                  " << env->solutionStatistics.numberOfProblemsQP
               << "\r\n";
    }

    if(env->solutionStatistics.numberOfProblemsQCQP > 0)
    {
        report << "  - QCQP problems                                " << env->solutionStatistics.numberOfProblemsQCQP
               << "\r\n";
    }

    if(env->solutionStatistics.numberOfProblemsOptimalMILP > 0)
    {
        report << "  - MILP problems, optimal                       "
               << env->solutionStatistics.numberOfProblemsOptimalMILP << "\r\n";
    }

    if(env->solutionStatistics.numberOfProblemsFeasibleMILP > 0)
    {
        report << "  - MILP problems, feasible                      "
               << env->solutionStatistics.numberOfProblemsFeasibleMILP << "\r\n";
    }

    if(env->solutionStatistics.numberOfProblemsOptimalMIQP > 0)
    {
        report << "  - MIQP problems, optimal                       "
               << env->solutionStatistics.numberOfProblemsOptimalMIQP << "\r\n";
    }

    if(env->solutionStatistics.numberOfProblemsFeasibleMIQP > 0)
    {
        report << "  - MIQP problems, feasible                      "
               << env->solutionStatistics.numberOfProblemsFeasibleMIQP << "\r\n";
    }

    report << "\r\n";

    if(env->solutionStatistics.numberOfExploredNodes > 0)
    {
        report << " Number of explored nodes:                       ";
        report << env->solutionStatistics.numberOfExploredNodes << "\r\n";

        report << "\r\n";
    }

    if(env->solutionStatistics.numberOfProblemsNLPInteriorPointSearch > 0
        || env->solutionStatistics.numberOfProblemsMinimaxLP > 0)
    {
        report << " Problems solved during interior point search: \r\n";

        if(env->solutionStatistics.numberOfProblemsNLPInteriorPointSearch > 0)
        {
            report << " - NLP problems:                                 ";
            report << env->solutionStatistics.numberOfProblemsNLPInteriorPointSearch << "\r\n";
        }

        if(env->solutionStatistics.numberOfProblemsMinimaxLP > 0)
        {
            report << " - LP problems:                                  ";
            report << env->solutionStatistics.numberOfProblemsMinimaxLP << "\r\n";
        }

        report << "\r\n";
    }

    if(env->solutionStatistics.numberOfProblemsFixedNLP > 0)
    {
        report << " Fixed primal NLP problems solved:               ";
        report << env->solutionStatistics.numberOfProblemsFixedNLP << "\r\n";
        report << "\r\n";
    }

    if(env->results->hasPrimalSolution())
    {
        report << fmt::format(" {:<48}{:d}",
                      "Number of primal solutions found:", env->solutionStatistics.numberOfFoundPrimalSolutions)
               << "\r\n";

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

            report << fmt::format(" - {:<46}{:d}", sourceDesc + ':', S.second) << "\r\n";
        }

        report << "\r\n";
    }

    for(auto& T : env->timing->timers)
    {
        T.stop();
        auto elapsed = T.elapsed();

        if(elapsed > 0)
        {
            report << fmt::format(" {:<48}{:g}", T.description + ':', elapsed) << "\r\n";
        }
    }

    env->output->outputInfo(report.str());
}

void Report::outputInteriorPointPreReport()
{
    std::stringstream report;

    report << "\r\n";
    report << "╶ Interior point search "
              "──────────────────────────────────────────────────────────────────────────────────────────────╴\r\n";
    report << "\r\n";

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