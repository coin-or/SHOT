/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Solver.h"

#include "DualSolver.h"
#include "PrimalSolver.h"
#include "Report.h"
#include "Results.h"
#include "Settings.h"
#include "TaskHandler.h"
#include "Timing.h"
#include "Utilities.h"

#ifdef HAS_GAMS
#include "ModelingSystem/ModelingSystemGAMS.h"
#endif
#ifdef HAS_AMPL
#include "ModelingSystem/ModelingSystemAMPL.h"
#endif
#include "ModelingSystem/ModelingSystemOSiL.h"

#include "SolutionStrategy/SolutionStrategySingleTree.h"
#include "SolutionStrategy/SolutionStrategyMultiTree.h"
#include "SolutionStrategy/SolutionStrategyMIQCQP.h"
#include "SolutionStrategy/SolutionStrategyNLP.h"

#include "Tasks/TaskPerformBoundTightening.h"
#include "Tasks/TaskReformulateProblem.h"

#include <map>

#ifdef HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std;
#endif

#ifdef HAS_STD_EXPERIMENTAL_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental;
#endif

#ifdef HAS_GUROBI
#include "gurobi_c++.h"
#endif

namespace SHOT
{

void Solver::createTimers()
{
    env->timing->createTimer("Total", "Total solution time");
    env->timing->startTimer("Total");

    env->timing->createTimer("ProblemInitialization", "- problem initialization");
    env->timing->createTimer("ProblemInitializationGAMS", "  - reading the model in GAMS");
    env->timing->createTimer("EigenvalueComputation", "- eigenvalue and eigenvector computation");
    env->timing->createTimer("ProblemReformulation", "- problem reformulation");
    env->timing->createTimer("ProblemReformulationEigenDecomp", "  - eigenvalue decomposition");
    env->timing->createTimer("ProblemReformulationLDLDecomp", "  - LDL decomposition");
    env->timing->createTimer("BoundTightening", "- bound tightening");
    env->timing->createTimer("BoundTighteningPOA", "  - initial outer approximation");
    env->timing->createTimer("BoundTighteningFBBTOriginal", "  - feasibility based (original problem)");
    env->timing->createTimer("BoundTighteningFBBTReformulated", "  - feasibility based (reformulated problem)");
}

Solver::Solver()
{
    env = std::make_shared<Environment>();

    env->output = std::make_shared<Output>();

    env->results = std::make_shared<Results>(env);
    env->timing = std::make_shared<Timing>(env);

    createTimers();

    env->settings = std::make_shared<Settings>(env->output);
    env->tasks = std::make_shared<TaskHandler>(env);
    env->callbacks = std::make_shared<CallbackHandler>(env);
    env->report = std::make_shared<Report>(env);

    env->dualSolver = std::make_shared<DualSolver>(env);
    env->primalSolver = std::make_shared<PrimalSolver>(env);
    initializeSettings();
}

Solver::Solver(std::shared_ptr<spdlog::sinks::sink> consoleSink)
{
    env = std::make_shared<Environment>();

    env->output = std::make_shared<Output>();
    if(consoleSink != nullptr)
        env->output->setConsoleSink(consoleSink);

    env->results = std::make_shared<Results>(env);
    env->timing = std::make_shared<Timing>(env);

    createTimers();

    env->settings = std::make_shared<Settings>(env->output);
    env->tasks = std::make_shared<TaskHandler>(env);
    env->callbacks = std::make_shared<CallbackHandler>(env);
    env->report = std::make_shared<Report>(env);

    env->dualSolver = std::make_shared<DualSolver>(env);
    env->primalSolver = std::make_shared<PrimalSolver>(env);
    initializeSettings();
}

Solver::Solver(EnvironmentPtr envPtr) : env(envPtr)
{
    // The environment may come from somewhere that does not register callbacks
    if(!env->callbacks)
        env->callbacks = std::make_shared<CallbackHandler>(env);

    initializeSettings();
}

Solver::~Solver() = default;

EnvironmentPtr Solver::getEnvironment() { return env; }

bool Solver::setOptionsFromFile(std::string fileName)
{
    bool result = true;
    try
    {
        std::string fileContents;
        std::string fileExtension = fs::filesystem::path(fileName).extension().string();

        if(fileExtension == ".xml" || fileExtension == ".osol")
        {
            fileContents = Utilities::getFileAsString(fileName);

            result = env->settings->readSettingsFromOSoL(fileContents);

            verifySettings();
        }
        else if(fileExtension == ".opt")
        {
            fileContents = Utilities::getFileAsString(fileName);
            result = env->settings->readSettingsFromString(fileContents);
        }
        else
        {
            env->output->outputError(
                " Error when reading options from \"" + fileName + "\". File extension must be osol, xml or opt.");
            result = false;
        }
    }
    catch(const std::exception& e)
    {
        env->output->outputError(" Error when reading options from \"" + fileName + "\"", e.what());
        result = false;
    }

    env->settings->updateSetting("Input.OptionsFile", fileName);

    env->output->outputDebug(" Options read from file \"" + fileName + "\"");

    return (result);
}

bool Solver::setOptionsFromString(std::string options)
{
    bool status = env->settings->readSettingsFromString(options);

    env->output->outputDebug(" Options read.");

    return (status);
}

bool Solver::setOptionsFromOSoL(std::string options)
{
    bool status = env->settings->readSettingsFromOSoL(options);

    verifySettings();

    env->output->outputDebug(" Options read.");

    return (status);
}

std::string Solver::getSettingsAsMarkup() { return (env->settings->getSettingsAsMarkup()); }

bool Solver::setLogFile(std::string filename)
{
    env->output->setFileSink(filename);
    return (true);
}

void Solver::updateLogLevels()
{
    env->output->setLogLevels(static_cast<E_LogLevel>(env->settings->getSetting<int>("Output.Console.LogLevel")),
        static_cast<E_LogLevel>(env->settings->getSetting<int>("Output.File.LogLevel")));
}

bool Solver::setProblem(std::string fileName)
{
    // The solution time starts here and not when the solver was created, since the time in between is spent in the
    // caller's code, e.g., waiting between the cells of a notebook, and would count against the time limit
    env->timing->restartTimer("Total");

    if(!fs::filesystem::exists(fileName))
    {
        env->output->outputError(" Problem file \"" + fileName + "\" does not exist.");

        return (false);
    }

    fs::filesystem::path problemFile(fileName);

    if(!problemFile.has_extension())
    {
        env->output->outputError(" Problem file \"" + fileName + "\" does not specify a file extension.");

        return (false);
    }

    fs::filesystem::path problemExtension = problemFile.extension();
    fs::filesystem::path problemPath = problemFile.parent_path();

    env->settings->updateSetting("Input.ProblemFile", fs::filesystem::absolute(problemFile).string());

    // Removes path
    fs::filesystem::path problemName = problemFile.stem();
    env->settings->updateSetting("Input.ProblemName", problemName.string());
    env->settings->updateSetting("Input.ProblemFile", fs::filesystem::absolute(problemFile).string());

    if(!setDebugPath())
        return (false);

    // Sets the result path
    if(static_cast<ES_OutputDirectory>(env->settings->getSetting<int>("Output.OutputDirectory"))
        == ES_OutputDirectory::Program)
    {
        env->settings->updateSetting("Output.ResultPath", fs::filesystem::current_path().string());
    }
    else
    {
        env->settings->updateSetting("Output.ResultPath", problemPath.string());
    }

    if(env->settings->getSetting<bool>("Output.Debug.Enable"))
    {
        initializeDebugMode();
    }

    // Do not do convexifying reformulations if the problem is assumed to be convex
    if(env->settings->getSetting<bool>("Model.Convexity.AssumeConvex"))
    {
        env->settings->updateSetting("Model.Reformulation.Bilinear.IntegerFormulation",
            (int)ES_ReformulateBilinearInteger::No, E_SettingPriority::RecommendedInternal);

        env->settings->updateSetting("Model.Reformulation.Monomials.Formulation",
            (int)ES_ReformulationBinaryMonomials::None, E_SettingPriority::RecommendedInternal);
    }

#ifndef HAS_GAMS
    if(problemExtension == ".gms")
    {
        env->output->outputError(" SHOT has not been compiled with support for GAMS files.");
        return (false);
    }
#endif

#ifndef HAS_AMPL
    if(problemExtension == ".nl")
    {
        env->output->outputError(" SHOT has not been compiled with support for AMPL .nl files.");
        return (false);
    }
#endif

#ifdef HAS_CBC
    if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Cbc)
    {
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy",
            (int)ES_QuadraticProblemStrategy::Nonlinear, E_SettingPriority::SolverCompatibility);
    }
#endif

#ifdef HAS_HIGHS
    if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Highs)
    {
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy",
            (int)ES_QuadraticProblemStrategy::Nonlinear, E_SettingPriority::SolverCompatibility);
    }
#endif

    try
    {
        if(problemExtension == ".osil" || problemExtension == ".xml")
        {
            env->report->outputModelingSystemReport(ES_ModelingSystem::OSiL, fileName);

            auto modelingSystem = std::make_shared<ModelingSystemOSiL>(env);
            ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

            if(modelingSystem->createProblem(problem, fileName) != E_ProblemCreationStatus::NormalCompletion)
            {
                return (false);
            }

            env->modelingSystem = modelingSystem;
            env->problem = problem;

            env->settings->updateSetting("Input.ModelingSystem", static_cast<int>(ES_ModelingSystem::OSiL));
        }

#ifdef HAS_AMPL
        if(problemExtension == ".nl")
        {
            env->report->outputModelingSystemReport(ES_ModelingSystem::AMPL, fileName);

            auto modelingSystem = std::make_shared<ModelingSystemAMPL>(env);
            ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

            if(modelingSystem->createProblem(problem, fileName) != E_ProblemCreationStatus::NormalCompletion)

            {
                return (false);
            }

            env->modelingSystem = modelingSystem;
            env->problem = problem;

            env->settings->updateSetting("Input.ModelingSystem", static_cast<int>(ES_ModelingSystem::AMPL));
        }
#endif

#ifdef HAS_GAMS
        if(problemExtension == ".gms")
        {
            env->report->outputModelingSystemReport(ES_ModelingSystem::GAMS, fileName);

            auto modelingSystem = std::make_shared<SHOT::ModelingSystemGAMS>(env);
            SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

            if(modelingSystem->createProblem(problem, fileName, E_GAMSInputSource::ProblemFile)
                != E_ProblemCreationStatus::NormalCompletion)
            {
                return (false);
            }

            env->modelingSystem = modelingSystem;
            env->problem = problem;

            // The levels of the variables are a starting point, which is given to the primal solver as when SHOT is
            // called from GAMS (EntryPointsGAMS.cpp) and as the initial values of an AMPL model
            VectorDouble variableStarts(gmoN(modelingSystem->modelingObject));
            gmoGetVarL(modelingSystem->modelingObject, variableStarts.data());
            env->primalSolver->addPrimalSolutionCandidate(
                variableStarts, E_PrimalSolutionSource::ExternalPrimalSolution, 0);

            env->settings->updateSetting("Input.ModelingSystem", static_cast<int>(ES_ModelingSystem::GAMS));
        }
#endif

        if(env->problem->name == "")
            env->problem->name = problemName.string();

#ifdef SIMPLE_OUTPUT_CHARS
        env->output->outputInfo(
            "- Bound tightening "
            "----------------------------------------------------------------------------------------------------");
#else
        env->output->outputInfo(
            "- Bound tightening "
            "───────────────────────────────────────────────────────────────────────────────────────────────────╴");
#endif

        if(env->settings->getSetting<bool>("Dual.MIP.CutOff.UseInitialValue")
            && std::abs(env->settings->getSetting<double>("Dual.MIP.CutOff.InitialValue")) < SHOT_DBL_MAX)
        {
            env->dualSolver->cutOffToUse = env->settings->getSetting<double>("Dual.MIP.CutOff.InitialValue");
            env->dualSolver->useCutOff = true;
            env->output->outputDebug(
                fmt::format("  Setting user defined cutoff value to {}.", env->dualSolver->cutOffToUse));
        }
        else
        {
            env->dualSolver->cutOffToUse = env->results->getPrimalBound();
        }

        auto taskPerformBoundTightening = std::make_unique<TaskPerformBoundTightening>(env, env->problem);
        taskPerformBoundTightening->run();

        setConvexityBasedSettingsPreReformulation();
        verifySettings();

        auto taskReformulateProblem = std::make_unique<TaskReformulateProblem>(env);
        taskReformulateProblem->run();

        if(env->problem->objectiveFunction->properties.isMinimize)
        {
            env->results->setDualBound(SHOT_DBL_MIN);

            if(!env->results->hasPrimalSolution())
                env->results->setPrimalBound(SHOT_DBL_MAX);
        }
        else
        {
            env->results->setDualBound(SHOT_DBL_MAX);

            if(!env->results->hasPrimalSolution())
                env->results->setPrimalBound(SHOT_DBL_MIN);
        }

        if(env->settings->getSetting<bool>("Output.Debug.Enable"))
        {
            fs::filesystem::path problemFilename(env->settings->getSetting<std::string>("Output.Debug.Path"));
            problemFilename /= "originalproblem.txt";

            std::stringstream problemText;
            problemText << env->problem;

            Utilities::writeStringToFile(problemFilename.string(), problemText.str());
        }
    }
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format(" Error when reading problem from \"{0}\"", e.what()));

        return (false);
    }

    setConvexityBasedSettings();
    verifySettings();

    bool isStrategySelected = selectStrategy();

    // The time until solveProblem() is called, e.g., in another cell of a notebook, is not part of the solution time
    env->timing->stopTimer("Total");

    return (isStrategySelected);
}

bool Solver::setProblem(
    SHOT::ProblemPtr problem, SHOT::ProblemPtr reformulatedProblem, SHOT::ModelingSystemPtr modelingSystem)
{
    // The solution time starts here for a problem built through the API, e.g. in Python, since the time since the
    // solver was created is spent in the caller's code, or waiting between the cells of a notebook, and would count
    // against the time limit. A caller that read the problem itself, as the GAMS entry point does, has timed that as
    // the problem initialization, which is kept in the solution time as when reading a problem file.
    if(env->timing->getElapsedTime("ProblemInitialization") == 0.0)
        env->timing->restartTimer("Total");

    env->modelingSystem = modelingSystem;
    env->problem = problem;

    if(!problem->objectiveFunction)
    {
        env->output->outputError(" Problem has no objective function. The solver cannot proceed correctly.");
        return false;
    }

    if(problem->allVariables.size() == 0)
    {
        env->output->outputError(" Problem has no variables. The solver cannot proceed correctly.");
        return false;
    }

    env->settings->updateSetting("Input.ProblemName", problem->name);

    if(!setDebugPath())
        return (false);

    if(env->settings->getSetting<bool>("Output.Debug.Enable"))
    {
        initializeDebugMode();
    }

    // Do not do convexifying reformulations if the problem is assumed to be convex
    if(env->settings->getSetting<bool>("Model.Convexity.AssumeConvex"))
    {
        env->settings->updateSetting("Model.Reformulation.Bilinear.IntegerFormulation",
            (int)ES_ReformulateBilinearInteger::No, E_SettingPriority::RecommendedInternal);

        env->settings->updateSetting("Model.Reformulation.Monomials.Formulation",
            (int)ES_ReformulationBinaryMonomials::None, E_SettingPriority::RecommendedInternal);
    }

#ifdef HAS_CBC
    if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Cbc)
    {
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy",
            (int)ES_QuadraticProblemStrategy::Nonlinear, E_SettingPriority::SolverCompatibility);
    }
#endif

#ifdef HAS_HIGHS
    if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Highs)
    {
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy",
            (int)ES_QuadraticProblemStrategy::Nonlinear, E_SettingPriority::SolverCompatibility);
    }
#endif

    // The cutoff the dual solver starts from, as in the overload reading the problem from a file. Without this
    // Dual.MIP.CutOff.InitialValue would silently have no effect on this path.
    if(env->settings->getSetting<bool>("Dual.MIP.CutOff.UseInitialValue")
        && std::abs(env->settings->getSetting<double>("Dual.MIP.CutOff.InitialValue")) < SHOT_DBL_MAX)
    {
        env->dualSolver->cutOffToUse = env->settings->getSetting<double>("Dual.MIP.CutOff.InitialValue");
        env->dualSolver->useCutOff = true;
        env->output->outputDebug(
            fmt::format("  Setting user defined cutoff value to {}.", env->dualSolver->cutOffToUse));
    }
    else
    {
        env->dualSolver->cutOffToUse = env->results->getPrimalBound();
    }

    // Bound tightening is performed here as well as in the overload that reads the problem from a file, so that
    // a problem provided directly through the API (e.g. from the Python interface or the GAMS entry point) is
    // treated the same way. It is controlled by Model.BoundTightening.FeasibilityBased.Use like everywhere else.
    //
    // It is skipped when the caller supplies its own reformulated problem: no reformulation is performed in that
    // case either, and the nested solver created by NLPSolverSHOT -- which the bound tightening task itself
    // creates when Model.BoundTightening.InitialPOA.Use is active -- passes its problem that way, so it would
    // otherwise recurse indefinitely.
    if(!reformulatedProblem)
    {
        auto taskPerformBoundTightening = std::make_unique<TaskPerformBoundTightening>(env, env->problem);
        taskPerformBoundTightening->run();
    }

    setConvexityBasedSettingsPreReformulation();
    verifySettings();

    if(reformulatedProblem)
    {
        env->reformulatedProblem = reformulatedProblem;
    }
    else
    {
        auto taskReformulateProblem = std::make_unique<TaskReformulateProblem>(env);
        taskReformulateProblem->run();
    }

    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        env->results->setDualBound(SHOT_DBL_MIN);

        if(!env->results->hasPrimalSolution())
            env->results->setPrimalBound(SHOT_DBL_MAX);
    }
    else
    {
        env->results->setDualBound(SHOT_DBL_MAX);

        if(!env->results->hasPrimalSolution())
            env->results->setPrimalBound(SHOT_DBL_MIN);
    }

    // Written after the reformulation, as in the overload reading the problem from a file, so that the dumped
    // problem reflects the bound tightening performed above.
    if(env->settings->getSetting<bool>("Output.Debug.Enable"))
    {
        fs::filesystem::path filename(env->settings->getSetting<std::string>("Output.Debug.Path"));
        filename /= "originalproblem.txt";

        std::stringstream problemText;
        problemText << env->problem;

        Utilities::writeStringToFile(filename.string(), problemText.str());
    }

    setConvexityBasedSettings();
    verifySettings();

    bool isStrategySelected = selectStrategy();

    // The time until solveProblem() is called, e.g., in another cell of a notebook, is not part of the solution time
    env->timing->stopTimer("Total");

    return (isStrategySelected);
}

bool Solver::selectStrategy()
{
    try
    {
        if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Cbc
            || static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Highs)
        {
            // The reformulated problem can have discrete variables also when the original problem has none, e.g., the
            // binary variables that bound absolute values from above
            auto problem = (env->reformulatedProblem) ? env->reformulatedProblem : env->problem;

            if(problem->properties.numberOfDiscreteVariables == 0
                && problem->properties.numberOfSemicontinuousVariables == 0)
            {
                env->output->outputDebug(" Using continuous problem solution strategy.");
                solutionStrategy = std::make_unique<SolutionStrategyNLP>(env);
                env->results->usedSolutionStrategy = E_SolutionStrategy::NLP;
            }
            else
            {
                solutionStrategy = std::make_unique<SolutionStrategyMultiTree>(env);
                isProblemInitialized = true;
                env->results->usedSolutionStrategy = E_SolutionStrategy::MultiTree;
            }

            return (true);
        }

        auto quadraticStrategy = static_cast<ES_QuadraticProblemStrategy>(
            env->settings->getSetting<int>("Model.Reformulation.Quadratics.Strategy"));
        bool useQuadraticConstraints
            = (quadraticStrategy >= ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained);
        bool useQuadraticObjective
            = (useQuadraticConstraints || quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticObjective);

        // Select from the model we actually solve: quadratic terms can disappear during linearization.
        const auto& properties = env->reformulatedProblem->properties;
        bool isConvex = properties.convexity == E_ProblemConvexity::Convex;

        if(isConvex && (useQuadraticObjective || useQuadraticConstraints) && properties.isMIQPProblem)
        // Convex MIQP problem
        {
            env->settings->updateSetting(
                "Output.Console.DualSolver.Show", true, E_SettingPriority::RecommendedInternal);
            env->output->outputDebug(" Using convex MIQP solution strategy.");
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQP;
        }
        else if(isConvex && (useQuadraticObjective || useQuadraticConstraints) && properties.isQPProblem)
        // Convex QP problem
        {
            env->output->outputDebug(" Using convex QP solution strategy.");
            env->settings->updateSetting(
                "Output.Console.DualSolver.Show", true, E_SettingPriority::RecommendedInternal);
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQP;
        }
        // Convex MIQCQP problem
        else if(isConvex && useQuadraticConstraints && properties.isMIQCQPProblem)
        {
            env->output->outputDebug(" Using convex MIQCQP solution strategy.");
            env->settings->updateSetting(
                "Output.Console.DualSolver.Show", true, E_SettingPriority::RecommendedInternal);
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQCQP;
        }
        // Convex QCQP problem
        else if(isConvex && useQuadraticConstraints && properties.isQCQPProblem)
        {
            env->output->outputDebug(" Using convex QCQP solution strategy.");
            env->settings->updateSetting(
                "Output.Console.DualSolver.Show", true, E_SettingPriority::RecommendedInternal);
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQCQP;
        }
        // Continuous linear and convex nonlinear problems use the continuous strategy. Linear mixed-integer
        // problems follow Dual.TreeStrategy below; the direct MIQCQP strategy is reserved for quadratic problems.
        else if(isConvex && (properties.isLPProblem || properties.isNLPProblem))
        {
            env->output->outputDebug(" Using continuous solution strategy.");
            solutionStrategy = std::make_unique<SolutionStrategyNLP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::NLP;
        }
        else
        {
            if(!properties.isDiscrete)
            {
                env->output->outputDebug(" Using continuous solution strategy.");
                solutionStrategy = std::make_unique<SolutionStrategyNLP>(env);
                env->results->usedSolutionStrategy = E_SolutionStrategy::NLP;
            }
            else
            {
                switch(static_cast<ES_TreeStrategy>(env->settings->getSetting<int>("Dual.TreeStrategy")))
                {
                case(ES_TreeStrategy::SingleTree):
                    env->output->outputDebug(" Using single-tree solution strategy.");
                    solutionStrategy = std::make_unique<SolutionStrategySingleTree>(env);
                    env->results->usedSolutionStrategy = E_SolutionStrategy::SingleTree;
                    env->dualSolver->isSingleTree = true;
                    break;
                case(ES_TreeStrategy::MultiTree):
                    env->output->outputDebug(" Using multi-tree solution strategy.");
                    solutionStrategy = std::make_unique<SolutionStrategyMultiTree>(env);
                    env->results->usedSolutionStrategy = E_SolutionStrategy::MultiTree;
                    break;
                default:
                    break;
                }
            }
        }

        // Want to show the output from Gurobi if Gurobi handles the whole problem
        if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Gurobi
            && ((useQuadraticObjective || useQuadraticConstraints)
                && (properties.isMIQPProblem || properties.isMIQCQPProblem || properties.isQCQPProblem
                    || properties.isQPProblem)))
        {
            env->settings->updateSetting(
                "Output.Console.DualSolver.Show", true, E_SettingPriority::RecommendedInternal);
        }
        isProblemInitialized = true;
    }
    catch(Exception& e)
    {
        env->output->outputCritical(e.what());
        return (false);
    }

    return (true);
}

bool Solver::solveProblem()
{
    // No strategy exists if setProblem() failed or was not called, e.g. when the MIP solver does not support the
    // problem
    if(solutionStrategy == nullptr)
    {
        env->output->outputCritical(" Cannot solve the problem since it has not been set successfully.");
        return (false);
    }

    // Paused at the end of setProblem()
    env->timing->startTimer("Total");

    // Verify settings in case they were changed after setProblem() was called
    verifySettings();

    // The debug mode can also be enabled, or its directory changed, after setProblem() was called
    if(env->settings->getSetting<bool>("Output.Debug.Enable")
        && !fs::filesystem::exists(env->settings->getSetting<std::string>("Output.Debug.Path")))
    {
        if(!setDebugPath())
            return (false);

        initializeDebugMode();
    }

    if(env->settings->getSetting<bool>("Output.Debug.Enable"))
    {
        fs::filesystem::path filename(env->settings->getSetting<std::string>("Output.Debug.Path"));
        filename /= "usedsettings.opt";

        auto usedSettings = env->settings->getSettingsAsString(false, false);

        Utilities::writeStringToFile(filename.string(), usedSettings);
    }

    // Callbacks cannot be registered or removed while the problem is solved
    struct SolvingGuard
    {
        bool& isSolving;
        SolvingGuard(bool& flag) : isSolving(flag) { isSolving = true; }
        ~SolvingGuard() { isSolving = false; }
    } solvingGuard(isSolving);

    isProblemSolved = solutionStrategy->solveProblem();

    // A failed callback overrides the reason SHOT terminated for, and its exception is passed on to the caller
    if(env->callbacks->hasFailed())
    {
        isProblemSolved = false;
        env->results->terminationReason = E_TerminationReason::Error;
        env->results->terminationReasonDescription
            = "Terminated since a callback failed: " + env->callbacks->getFailureMessage();
    }

    this->finalizeSolution();

    env->callbacks->rethrowFailure();

    return (isProblemSolved);
}

int Solver::registerCallback(E_CallbackLocation locations, CallbackFunction callback)
{
    if(isSolving)
        throw std::logic_error("Callbacks cannot be registered while the problem is solved.");

    return (env->callbacks->add(locations, std::move(callback)));
}

bool Solver::removeCallback(int handle)
{
    if(isSolving)
        throw std::logic_error("Callbacks cannot be removed while the problem is solved.");

    return (env->callbacks->remove(handle));
}

void Solver::finalizeSolution()
{
    if(env->modelingSystem)
        env->modelingSystem->finalizeSolution();
}

void Solver::outputSolverHeader() { env->report->outputSolverHeader(); }
void Solver::outputOptionsReport() { env->report->outputOptionsReport(); }
void Solver::outputProblemInstanceReport() { env->report->outputProblemInstanceReport(); }
void Solver::outputSolutionReport() { env->report->outputSolutionReport(); }

std::string Solver::getResultsOSrL() { return (env->results->getResultsOSrL()); }

std::string Solver::getOptionsOSoL()
{
    if(!env->settings->settingsInitialized)
        initializeSettings();

    return (env->settings->getSettingsAsOSoL());
}

std::string Solver::getOptions()
{
    if(!env->settings->settingsInitialized)
        initializeSettings();

    return (env->settings->getSettingsAsString(false, false));
}

std::string Solver::getResultsTrace() { return (env->results->getResultsTrace()); }

std::string Solver::getResultsSol() { return (env->results->getResultsSol()); }

// Creates a temporary debug directory if the debug mode is enabled and no directory is given
bool Solver::setDebugPath()
{
    if(!env->settings->getSetting<bool>("Output.Debug.Enable")
        || env->settings->getSetting<std::string>("Output.Debug.Path") != "")
        return (true);

    auto debugPath = Utilities::createTemporaryDirectory("SHOT_debug_");

    if(debugPath == "")
    {
        env->output->outputError(" Could not create debug directory.");
        return (false);
    }

    env->settings->updateSetting("Output.Debug.Path", debugPath);
    return (true);
}

void Solver::initializeDebugMode()
{
    auto debugPath = env->settings->getSetting<std::string>("Output.Debug.Path");
    fs::filesystem::path debugDir(debugPath);

    if(fs::filesystem::exists(debugDir))
    {
        env->output->outputDebug(" Debug directory " + debugPath + " already exists.");
    }
    else
    {
        if(fs::filesystem::create_directories(debugDir))
        {
            env->output->outputDebug(" Debug directory " + debugPath + " created.");
        }
        else
        {
            env->output->outputWarning(" Could not create debug directory.");
        }
    }

    if(env->settings->getSetting<std::string>("Input.ProblemFile") != "")
    {
        fs::filesystem::path source(
            fs::filesystem::canonical(env->settings->getSetting<std::string>("Input.ProblemFile")));

        try
        {
            fs::filesystem::copy_file(
                source, (debugDir / source.filename()), fs::filesystem::copy_options::overwrite_existing);
        }
        catch(const fs::filesystem::filesystem_error& e)
        {
            env->output->outputError(" Error when copying problem file to debug directory: ", e.what());
        }
    }
}

void Solver::verifySettings()
{
    env->output->setLogLevels(static_cast<E_LogLevel>(env->settings->getSetting<int>("Output.Console.LogLevel")),
        static_cast<E_LogLevel>(env->settings->getSetting<int>("Output.File.LogLevel")));

    // Checking for errors in NLP solver selection

    // The GAMS NLP solvers can only be used for problems from GAMS. The settings can be verified before the problem is
    // read, e.g. from an options file, and the modeling system is then not known yet.
    bool isNonGAMSProblem = env->problem
        && env->settings->getSetting<int>("Input.ModelingSystem") != static_cast<int>(ES_ModelingSystem::GAMS);

    // Returns why the NLP solver cannot be used, or an empty string if it can
    auto getReasonNLPSolverUnavailable = [&](ES_PrimalNLPSolver solver) -> std::string
    {
        switch(solver)
        {
        case ES_PrimalNLPSolver::Ipopt:
#ifdef HAS_IPOPT
            return ("");
#else
            return ("SHOT has not been compiled with support for the Ipopt NLP solver");
#endif
        case ES_PrimalNLPSolver::Conopt:
#ifdef HAS_CONOPT
            return ("");
#else
            return ("SHOT has not been compiled with support for CONOPT");
#endif
        case ES_PrimalNLPSolver::GAMS:
#ifdef HAS_GAMS
            return (isNonGAMSProblem ? "the GAMS NLP solvers cannot be used for problems not originating from GAMS "
                                       "(OSiL, nl, or API-built problems)"
                                     : "");
#else
            return ("SHOT has not been compiled with support for GAMS NLP solvers");
#endif
        case ES_PrimalNLPSolver::Uno:
#ifdef HAS_UNO
            return ("");
#else
            return ("SHOT has not been compiled with support for the Uno NLP solver");
#endif
        case ES_PrimalNLPSolver::SHOT:
            return ("");
        default:
            return ("the NLP solver is not valid");
        }
    };

    auto NLPSolver = static_cast<ES_PrimalNLPSolver>(env->settings->getSetting<int>("Primal.FixedInteger.Solver"));

    if(auto reason = getReasonNLPSolverUnavailable(NLPSolver); !reason.empty())
    {
        // The default solver, Ipopt, is replaced without a warning if it is not available
        bool isChosenByUser
            = env->settings->getSettingPriority("Primal.FixedInteger.Solver") > E_SettingPriority::Default;

        for(auto alternative : { ES_PrimalNLPSolver::Ipopt, ES_PrimalNLPSolver::Conopt, ES_PrimalNLPSolver::GAMS,
                ES_PrimalNLPSolver::Uno, ES_PrimalNLPSolver::SHOT })
        {
            if(!getReasonNLPSolverUnavailable(alternative).empty())
                continue;

            env->settings->updateSetting(
                "Primal.FixedInteger.Solver", static_cast<int>(alternative), E_SettingPriority::SolverCompatibility);

            std::map<ES_PrimalNLPSolver, std::string> names = { { ES_PrimalNLPSolver::Ipopt, "Ipopt" },
                { ES_PrimalNLPSolver::Conopt, "CONOPT" }, { ES_PrimalNLPSolver::GAMS, "the GAMS NLP solvers" },
                { ES_PrimalNLPSolver::Uno, "Uno" }, { ES_PrimalNLPSolver::SHOT, "SHOT" } };

            auto message = fmt::format(
                " The selected NLP solver cannot be used since {}. Using {} instead.", reason, names[alternative]);

            if(isChosenByUser)
                env->output->outputWarning(message);
            else
                env->output->outputDebug(message);

            break;
        }
    }

#ifdef HAS_GAMS
    // This is checked after the NLP solver may have been replaced with a GAMS solver above
    if((static_cast<ES_PrimalNLPSolver>(env->settings->getSetting<int>("Primal.FixedInteger.Solver"))
           == ES_PrimalNLPSolver::GAMS)
        && (static_cast<ES_PrimalNLPProblemSource>(env->settings->getSetting<int>("Primal.FixedInteger.SourceProblem"))
            != ES_PrimalNLPProblemSource::OriginalProblem))
    {
        // The reformulated problem is the default, so the warning is only given if it has been selected
        if(env->settings->getSettingPriority("Primal.FixedInteger.SourceProblem") > E_SettingPriority::Default)
            env->output->outputWarning(" Cannot use GAMS NLP solvers when solving fixed NLP problems based on the "
                                       "reformulated model. Use Ipopt instead!");

        env->settings->updateSetting("Primal.FixedInteger.SourceProblem",
            static_cast<int>(ES_PrimalNLPProblemSource::OriginalProblem), E_SettingPriority::SolverCompatibility);
    }
#endif

    // Checking for errors in MIP solver selection

    auto solver = static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver"));
    bool MIPSolverDefined = false;
    double unboundedVariableBound = 1e50;

#ifdef HAS_CPLEX
    if(solver == ES_MIPSolver::Cplex)
    {
        MIPSolverDefined = true;
        unboundedVariableBound = 1e20;
    }
#endif

#ifdef HAS_GUROBI
    if(solver == ES_MIPSolver::Gurobi)
    {
        MIPSolverDefined = true;
        unboundedVariableBound = 1e20;
    }
#endif

#ifdef HAS_CBC
    if(solver == ES_MIPSolver::Cbc)
    {
        MIPSolverDefined = true;
        unboundedVariableBound = 1e50;

        // Some features are not available in Cbc
        env->settings->updateSetting(
            "Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree), E_SettingPriority::SolverCompatibility);
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy",
            static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear), E_SettingPriority::SolverCompatibility);
    }
#endif

#ifdef HAS_HIGHS
    if(solver == ES_MIPSolver::Highs)
    {
        MIPSolverDefined = true;
        unboundedVariableBound = 1e20;

        // Some features are not available in Highs
        env->settings->updateSetting(
            "Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree), E_SettingPriority::SolverCompatibility);
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy",
            static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear), E_SettingPriority::SolverCompatibility);
    }
#endif

    if(!MIPSolverDefined)
    {
        env->output->outputWarning(" SHOT has not been compiled with support for selected MIP solver.");

#ifdef HAS_GUROBI
        env->settings->updateSetting(
            "Dual.MIP.Solver", (int)ES_MIPSolver::Gurobi, E_SettingPriority::RecommendedInternal);
        unboundedVariableBound = 1e20;
#elif HAS_CPLEX
        env->settings->updateSetting(
            "Dual.MIP.Solver", (int)ES_MIPSolver::Cplex, E_SettingPriority::RecommendedInternal);
        unboundedVariableBound = 1e20;
#elif HAS_HIGHS
        env->settings->updateSetting(
            "Dual.MIP.Solver", (int)ES_MIPSolver::Highs, E_SettingPriority::RecommendedInternal);
        unboundedVariableBound = 1e20;
#elif HAS_CBC
        env->settings->updateSetting("Dual.MIP.Solver", (int)ES_MIPSolver::Cbc, E_SettingPriority::RecommendedInternal);
        unboundedVariableBound = 1e50;

#else
        env->output->outputCritical(" SHOT has not been compiled with support for any MIP solver.");
#endif
    }

    // Updating max bound setting for unbounded variables
    double minLB = env->settings->getSetting<double>("Model.Variables.Continuous.MinimumLowerBound");
    double maxUB = env->settings->getSetting<double>("Model.Variables.Continuous.MaximumUpperBound");

    if(minLB < -unboundedVariableBound)
    {
        env->settings->updateSetting("Model.Variables.Continuous.MinimumLowerBound", -unboundedVariableBound,
            E_SettingPriority::RecommendedInternal);
    }

    if(maxUB > unboundedVariableBound)
    {
        env->settings->updateSetting("Model.Variables.Continuous.MaximumUpperBound", unboundedVariableBound,
            E_SettingPriority::RecommendedInternal);
    }

    // Set correct iteration detail output when showing dual solver output
    if(env->settings->getSetting<bool>("Output.Console.DualSolver.Show"))
        env->settings->updateSetting("Output.Console.Iteration.Detail",
            static_cast<int>(ES_IterationOutputDetail::Full), E_SettingPriority::RecommendedInternal);

    // Set correct iteration detail output when showing primal solver output
    if(env->settings->getSetting<bool>("Output.Console.PrimalSolver.Show"))
        env->settings->updateSetting("Output.Console.Iteration.Detail",
            static_cast<int>(ES_IterationOutputDetail::Full), E_SettingPriority::RecommendedInternal);
}

void Solver::setConvexityBasedSettingsPreReformulation()
{
    if(env->settings->getSetting<bool>("Strategy.UseRecommendedSettings"))
    {
        if(env->problem->properties.convexity != E_ProblemConvexity::Convex)
        {
            // env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy", 0);

#if defined(HAS_GUROBI) && GRB_VERSION_MAJOR >= 9
            // Activate Gurobi nonconvex MIQCQP solver for all nonconvex quadratic terms by default
            if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Gurobi)
                env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy",
                    (int)ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained,
                    E_SettingPriority::RecommendedInternal);
#endif
        }
    }
}

void Solver::setConvexityBasedSettings()
{
    // Nonconvex problems are always solved with the multi-tree strategy
    if(env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex)
    {
        if(env->settings->getSetting<int>("Dual.TreeStrategy") == static_cast<int>(ES_TreeStrategy::SingleTree)
            && env->settings->getSettingPriority("Dual.TreeStrategy") > E_SettingPriority::Default)
            env->output->outputWarning(
                " The problem is nonconvex, so the multi-tree strategy is used instead of the single-tree strategy.");

        env->settings->updateSetting(
            "Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree), E_SettingPriority::SolverCompatibility);
    }

    if(env->settings->getSetting<bool>("Strategy.UseRecommendedSettings"))
    {
        if(env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex)
        {
            env->settings->updateSetting(
                "Dual.ESH.InteriorPoint.CuttingPlane.IterationLimit", 50, E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting(
                "Dual.HyperplaneCuts.ConstraintSelectionFactor", 1.0, E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting(
                "Dual.HyperplaneCuts.UseIntegerCuts", true, E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting(
                "Dual.MIP.SolutionLimit.Initial", SHOT_INT_MAX, E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting("Dual.Relaxation.Use", false, E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting("Primal.FixedInteger.CallStrategy", 0, E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting(
                "Primal.FixedInteger.Enumeration.UseInitially", true, E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Primal.FixedInteger.Source", 0, E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting(
                "Primal.FixedInteger.OnlyUniqueIntegerCombinations", false, E_SettingPriority::RecommendedInternal);

#ifdef HAS_CPLEX

            if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Cplex)
            {
                if(env->reformulatedProblem->objectiveFunction->properties.classification
                        == E_ObjectiveFunctionClassification::Quadratic
                    || env->reformulatedProblem->properties.numberOfQuadraticConstraints > 0)
                    env->settings->updateSetting(
                        "Subsolver.Cplex.OptimalityTarget", 3, E_SettingPriority::RecommendedInternal);
            }

#endif
        }

        // When the reformulated problem is a MILP problem, e.g., when all nonlinear terms are products of binary
        // variables, the MIP solver has already solved the LP problem that remains with the discrete variables fixed.
        // Such an NLP problem can also be much larger than the original problem, since it contains the auxiliary
        // variables and the linearizations.
        if(env->reformulatedProblem->properties.isMILPProblem)
            env->settings->updateSetting("Primal.FixedInteger.Use", false, E_SettingPriority::RecommendedInternal);
    }
}

VectorString Solver::getSettingIdentifiers(E_SettingType type) { return (env->settings->getSettingIdentifiers(type)); }

double Solver::getCurrentDualBound() { return (env->results->getCurrentDualBound()); }

double Solver::getGlobalDualBound() { return (env->results->getGlobalDualBound()); }

double Solver::getPrimalBound() { return (env->results->getPrimalBound()); }

double Solver::getAbsoluteObjectiveGap() { return (env->results->getAbsoluteGlobalObjectiveGap()); }

double Solver::getRelativeObjectiveGap() { return (env->results->getRelativeGlobalObjectiveGap()); }

bool Solver::hasPrimalSolution() { return (isProblemSolved && env->results->hasPrimalSolution() ? true : false); }

PrimalSolution Solver::getPrimalSolution()
{
    if(hasPrimalSolution())
        return (env->results->primalSolutions[0]);

    throw NoPrimalSolutionException("Can not get primal solution since none has been found.");
}

std::vector<PrimalSolution> Solver::getPrimalSolutions() { return (env->results->primalSolutions); }

E_TerminationReason Solver::getTerminationReason() { return (env->results->terminationReason); }

E_ModelReturnStatus Solver::getModelReturnStatus() { return (env->results->getModelReturnStatus()); }

std::vector<ES_ModelingSystem> Solver::getSupportedModelingSystems()
{
    std::vector<ES_ModelingSystem> systems;
    systems.push_back(ES_ModelingSystem::OSiL); // Always available
#ifdef HAS_GAMS
    systems.push_back(ES_ModelingSystem::GAMS);
#endif
#ifdef HAS_AMPL
    systems.push_back(ES_ModelingSystem::AMPL);
#endif
    return systems;
}

std::vector<ES_MIPSolver> Solver::getSupportedMIPSolvers()
{
    std::vector<ES_MIPSolver> solvers;
#ifdef HAS_CPLEX
    solvers.push_back(ES_MIPSolver::Cplex);
#endif
#ifdef HAS_GUROBI
    solvers.push_back(ES_MIPSolver::Gurobi);
#endif
#ifdef HAS_CBC
    solvers.push_back(ES_MIPSolver::Cbc);
#endif
#ifdef HAS_HIGHS
    solvers.push_back(ES_MIPSolver::Highs);
#endif
    return solvers;
}

std::vector<ES_PrimalNLPSolver> Solver::getSupportedNLPSolvers()
{
    std::vector<ES_PrimalNLPSolver> solvers;
    solvers.push_back(ES_PrimalNLPSolver::SHOT); // Always available
#ifdef HAS_CONOPT
    solvers.push_back(ES_PrimalNLPSolver::Conopt);
#endif
#ifdef HAS_IPOPT
    solvers.push_back(ES_PrimalNLPSolver::Ipopt);
#endif
#ifdef HAS_GAMS
    solvers.push_back(ES_PrimalNLPSolver::GAMS);
#endif
#ifdef HAS_UNO
    solvers.push_back(ES_PrimalNLPSolver::Uno);
#endif
    return solvers;
}

bool Solver::hasModelingSystem(ES_ModelingSystem format)
{
    switch(format)
    {
    case ES_ModelingSystem::OSiL:
        return true; // Always available
    case ES_ModelingSystem::GAMS:
#ifdef HAS_GAMS
        return true;
#else
        return false;
#endif
    case ES_ModelingSystem::AMPL:
#ifdef HAS_AMPL
        return true;
#else
        return false;
#endif
    default:
        return false;
    }
}

bool Solver::hasMIPSolver(ES_MIPSolver solver)
{
    switch(solver)
    {
    case ES_MIPSolver::Cplex:
#ifdef HAS_CPLEX
        return true;
#else
        return false;
#endif
    case ES_MIPSolver::Gurobi:
#ifdef HAS_GUROBI
        return true;
#else
        return false;
#endif
    case ES_MIPSolver::Cbc:
#ifdef HAS_CBC
        return true;
#else
        return false;
#endif
    default:
        return false;
    }
}

bool Solver::hasNLPSolver(ES_PrimalNLPSolver solver)
{
    switch(solver)
    {
    case ES_PrimalNLPSolver::Conopt:
#ifdef HAS_CONOPT
        return true;
#else
        return false;
#endif
    case ES_PrimalNLPSolver::SHOT:
        return true; // Always available
    case ES_PrimalNLPSolver::Ipopt:
#ifdef HAS_IPOPT
        return true;
#else
        return false;
#endif
    case ES_PrimalNLPSolver::GAMS:
#ifdef HAS_GAMS
        return true;
#else
        return false;
#endif
    case ES_PrimalNLPSolver::Uno:
#ifdef HAS_UNO
        return true;
#else
        return false;
#endif
    default:
        return false;
    }
}

} // namespace SHOT
