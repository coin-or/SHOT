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

Solver::Solver()
{
    env = std::make_shared<Environment>();

    env->output = std::make_shared<Output>();

    env->results = std::make_shared<Results>(env);
    env->timing = std::make_shared<Timing>(env);

    env->timing->createTimer("Total", "Total solution time");
    env->timing->startTimer("Total");

    env->timing->createTimer("ProblemInitialization", "- problem initialization");
    env->timing->createTimer("EigenvalueComputation", "- eigenvalue and eigenvector computation");
    env->timing->createTimer("ProblemReformulation", "- problem reformulation");
    env->timing->createTimer("ProblemReformulationEigenDecomp", "  - eigenvalue decomposition");
    env->timing->createTimer("ProblemReformulationLDLDecomp", "  - LDL decomposition");
    env->timing->createTimer("BoundTightening", "- bound tightening");
    env->timing->createTimer("BoundTighteningPOA", "  - initial outer approximation");
    env->timing->createTimer("BoundTighteningFBBTOriginal", "  - feasibility based (original problem)");
    env->timing->createTimer("BoundTighteningFBBTReformulated", "  - feasibility based (reformulated problem)");

    env->settings = std::make_shared<Settings>(env->output);
    env->tasks = std::make_shared<TaskHandler>(env);
    env->events = std::make_shared<EventHandler>(env);
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

    env->timing->createTimer("Total", "Total solution time");
    env->timing->startTimer("Total");

    env->timing->createTimer("ProblemInitialization", "- problem initialization");
    env->timing->createTimer("ProblemReformulation", "- problem reformulation");
    env->timing->createTimer("BoundTightening", "- bound tightening");
    env->timing->createTimer("BoundTighteningFBBT", "  - feasibility based");
    env->timing->createTimer("BoundTighteningFBBTOriginal", "  - feasibility based (original problem");
    env->timing->createTimer("BoundTighteningFBBTReformulated", "  - feasibility based (reformulated problem");
    env->timing->createTimer("EigenvalueComputation", "- eigenvalue and eigenvector computation");

    env->settings = std::make_shared<Settings>(env->output);
    env->tasks = std::make_shared<TaskHandler>(env);
    env->events = std::make_shared<EventHandler>(env);
    env->report = std::make_shared<Report>(env);

    env->dualSolver = std::make_shared<DualSolver>(env);
    env->primalSolver = std::make_shared<PrimalSolver>(env);
    initializeSettings();
}

Solver::Solver(EnvironmentPtr envPtr) : env(envPtr) { initializeSettings(); }

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

    // Sets the debug path if not already set
    if(env->settings->getSetting<bool>("Output.Debug.Enable")
        && env->settings->getSetting<std::string>("Output.Debug.Path") == "")
    {
        if(auto debugPath = Utilities::createTemporaryDirectory("SHOT_debug_"); debugPath == "")
        {
            env->output->outputError(" Could not create debug directory.");
            return (false);
        }
        else
        {
            env->settings->updateSetting("Output.Debug.Path", debugPath);
        }
    }

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
        env->settings->updateSetting("Model.Reformulation.Bilinear.IntegerFormulation", (int)ES_ReformulateBilinearInteger::No);

        env->settings->updateSetting("Model.Reformulation.Monomials.Formulation", (int)ES_ReformulationBinaryMonomials::None);
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
    // TODO: figure out a better way to do this
    if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Cbc)
    {
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy", (int)ES_QuadraticProblemStrategy::Nonlinear);
    }
#endif

#ifdef HAS_HIGHS
    // TODO: figure out a better way to do this
    if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Highs)
    {
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy", (int)ES_QuadraticProblemStrategy::Nonlinear);
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

        if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
        {
            env->results->setDualBound(SHOT_DBL_MIN);
            env->results->setPrimalBound(SHOT_DBL_MAX);
        }
        else
        {
            env->results->setDualBound(SHOT_DBL_MAX);
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

    return (this->selectStrategy());
}

bool Solver::setProblem(
    SHOT::ProblemPtr problem, SHOT::ProblemPtr reformulatedProblem, SHOT::ModelingSystemPtr modelingSystem)
{
    env->modelingSystem = modelingSystem;
    env->problem = problem;

    env->settings->updateSetting("Input.ProblemName", problem->name);

    // Sets the debug path if not already set
    if(env->settings->getSetting<bool>("Output.Debug.Enable")
        && env->settings->getSetting<std::string>("Output.Debug.Path") == "")
    {
        if(auto debugPath = Utilities::createTemporaryDirectory("SHOT_debug_"); debugPath == "")
        {
            env->output->outputError(" Could not create debug directory.");
            return (false);
        }
        else
        {
            env->settings->updateSetting("Output.Debug.Path", debugPath);
        }
    }

    if(env->settings->getSetting<bool>("Output.Debug.Enable"))
    {
        initializeDebugMode();

        fs::filesystem::path filename(env->settings->getSetting<std::string>("Output.Debug.Path"));
        filename /= "originalproblem.txt";

        std::stringstream problem;
        problem << env->problem;

        Utilities::writeStringToFile(filename.string(), problem.str());
    }

    // Do not do convexifying reformulations if the problem is assumed to be convex
    if(env->settings->getSetting<bool>("Model.Convexity.AssumeConvex"))
    {
        env->settings->updateSetting("Model.Reformulation.Bilinear.IntegerFormulation", (int)ES_ReformulateBilinearInteger::No);

        env->settings->updateSetting("Model.Reformulation.Monomials.Formulation", (int)ES_ReformulationBinaryMonomials::None);
    }

#ifdef HAS_CBC
    // TODO: figure out a better way to do this
    if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Cbc)
    {
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy", (int)ES_QuadraticProblemStrategy::Nonlinear);
    }
#endif

#ifdef HAS_HIGHS
    // TODO: figure out a better way to do this
    if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Highs)
    {
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy", (int)ES_QuadraticProblemStrategy::Nonlinear);
    }
#endif

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

    if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
    {
        env->results->setDualBound(SHOT_DBL_MIN);
        env->results->setPrimalBound(SHOT_DBL_MAX);
    }
    else
    {
        env->results->setDualBound(SHOT_DBL_MAX);
        env->results->setPrimalBound(SHOT_DBL_MIN);
    }

    setConvexityBasedSettings();
    verifySettings();

    return (this->selectStrategy());
}

bool Solver::selectStrategy()
{
    try
    {
        if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Cbc
            || static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Highs)
        {
            if(env->problem->properties.numberOfDiscreteVariables == 0
                && env->problem->properties.numberOfSemicontinuousVariables == 0)
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

        bool isConvex = env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Convex;

        if(isConvex && (useQuadraticObjective || useQuadraticConstraints) && env->problem->properties.isMIQPProblem)
        // Convex MIQP problem
        {
            env->settings->updateSetting("Output.Console.DualSolver.Show", true);
            env->output->outputDebug(" Using convex MIQP solution strategy.");
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQP;
        }
        else if(isConvex && (useQuadraticObjective || useQuadraticConstraints) && env->problem->properties.isQPProblem)
        // Convex QP problem
        {
            env->output->outputDebug(" Using convex QP solution strategy.");
            env->settings->updateSetting("Output.Console.DualSolver.Show", true);
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQP;
        }
        // Convex MIQCQP problem
        else if(isConvex && useQuadraticConstraints && env->problem->properties.isMIQCQPProblem)
        {
            env->output->outputDebug(" Using convex MIQCQP solution strategy.");
            env->settings->updateSetting("Output.Console.DualSolver.Show", true);
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQCQP;
        }
        // Convex QCQP problem
        else if(isConvex && (useQuadraticConstraints || useQuadraticConstraints)
            && env->problem->properties.isQCQPProblem)
        {
            env->output->outputDebug(" Using convex QCQP solution strategy.");
            env->settings->updateSetting("Output.Console.DualSolver.Show", true);
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQCQP;
        }
        // MILP problem
        else if(env->problem->properties.isMILPProblem || env->problem->properties.isLPProblem)
        {
            env->output->outputDebug(" Using MILP solution strategy.");
            env->settings->updateSetting("Output.Console.DualSolver.Show", true);
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQP;
        }
        // NLP problem
        else if(isConvex && (env->problem->properties.isNLPProblem))
        {
            env->output->outputDebug(" Using continous solution strategy.");
            solutionStrategy = std::make_unique<SolutionStrategyNLP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::NLP;
        }
        else
        {
            if(!env->problem->properties.isDiscrete)
            {
                env->output->outputDebug(" Using multi-tree solution strategy.");
                solutionStrategy = std::make_unique<SolutionStrategyMultiTree>(env);
                env->results->usedSolutionStrategy = E_SolutionStrategy::MultiTree;
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
                && (env->problem->properties.isMIQPProblem || env->problem->properties.isMIQCQPProblem
                    || env->problem->properties.isQCQPProblem || env->problem->properties.isQPProblem)))
        {
            env->settings->updateSetting("Output.Console.DualSolver.Show", true);
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
    // Verify settings in case they were changed after setProblem() was called
    verifySettings();

    if(env->settings->getSetting<bool>("Output.Debug.Enable"))
    {
        fs::filesystem::path filename(env->settings->getSetting<std::string>("Output.Debug.Path"));
        filename /= "usedsettings.opt";

        auto usedSettings = env->settings->getSettingsAsString(false, false);

        Utilities::writeStringToFile(filename.string(), usedSettings);
    }

    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        env->results->setDualBound(SHOT_DBL_MIN);
        env->results->setPrimalBound(SHOT_DBL_MAX);
    }
    else
    {
        env->results->setDualBound(SHOT_DBL_MAX);
        env->results->setPrimalBound(SHOT_DBL_MIN);
    }

    assert(solutionStrategy != nullptr); /* would be NULL if setProblem failed */
    isProblemSolved = solutionStrategy->solveProblem();

    this->finalizeSolution();

    return (isProblemSolved);
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

void Solver::initializeSettings()
{
    if(env->settings->settingsInitialized)
    {
        env->output->outputWarning(" Warning! Settings have already been initialized. Ignoring new settings.");
        return;
    }

    std::string empty; // Used to create empty string options

    env->output->outputDebug(" Starting initialization of settings:");

    env->settings->createSettingGroup("Dual", "", "Dual strategy",
        "These settings control the various functionality of the dual strategy in SHOT, i.e., the polyhedral outer "
        "approximation utilizing the ESH or ECP algorithms.");

    // Dual strategy settings: ECP and ESH

    VectorString enumHyperplanePointStrategy;
    enumHyperplanePointStrategy.push_back("ESH");
    enumHyperplanePointStrategy.push_back("ECP");
    enumHyperplanePointStrategy.push_back("Only external (through callback)");
    env->settings->createSetting("Dual.CutStrategy", static_cast<int>(ES_HyperplaneCutStrategy::ESH),
        "Dual cut strategy", enumHyperplanePointStrategy, 0);
    enumHyperplanePointStrategy.clear();

    env->settings->createSettingGroup("Dual", "ESH", "Extended supporting hyperplane method",
        "These settings control various aspects of the ESH implementation, including the strategy to obtain the "
        "interior point.");

    // Dual strategy settings: Interior point search strategy

    VectorString enumInteriorPointStrategy;
    enumInteriorPointStrategy.push_back("Use internal strategy");
    enumInteriorPointStrategy.push_back("Only external (through callback)");
    env->settings->createSetting("Dual.ESH.InteriorPoint.Strategy",         static_cast<int>(ES_ESHInteriorPointStrategy::UseInternalStrategy),
        "Strategy for finding ESH interior points", enumInteriorPointStrategy, 0);
    enumInteriorPointStrategy.clear();

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.BitPrecision", 8,
        "Required termination bit precision for minimization subsolver", 1, 64, true);

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.ConstraintSelectionFactor", 0.25,
        "The fraction of violated constraints to generate cutting planes for", 0.0, 1.0);

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.IterationLimit", 100,
        "Iteration limit for minimax cutting plane solver", 1, SHOT_INT_MAX);

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.IterationLimitSubsolver", 100,
        "Iteration limit for minimization subsolver", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.TimeLimit", 10.0, "Time limit for minimax solver", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.Reuse", false, "Reuse valid cutting planes in main dual model");

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.TerminationToleranceAbs", 1.0,
        "Absolute termination tolerance between LP and linesearch objective", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.TerminationToleranceRel", 1.0,
        "Relative termination tolerance between LP and linesearch objective", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.ESH.InteriorPoint.MinimaxObjectiveLowerBound", -1e12,
        "Lower bound for minimax objective variable", SHOT_DBL_MIN, 0);

    env->settings->createSetting("Dual.ESH.InteriorPoint.MinimaxObjectiveUpperBound", 0.1,
        "Upper bound for minimax objective variable", SHOT_DBL_MIN, SHOT_DBL_MAX);

    VectorString enumAddPrimalPointAsInteriorPoint;
    enumAddPrimalPointAsInteriorPoint.push_back("No");
    enumAddPrimalPointAsInteriorPoint.push_back("Add as new");
    enumAddPrimalPointAsInteriorPoint.push_back("Replace old");
    enumAddPrimalPointAsInteriorPoint.push_back("Use avarage");
    env->settings->createSetting("Dual.ESH.InteriorPoint.UsePrimalSolution",         static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepBoth), "Utilize primal solution as interior point",
        enumAddPrimalPointAsInteriorPoint, 0);
    enumAddPrimalPointAsInteriorPoint.clear();

    env->settings->createSetting("Dual.ESH.Rootsearch.ConstraintTolerance", 1e-8,
        "Constraint tolerance for when not to add individual hyperplanes", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.ESH.Rootsearch.UniqueConstraints", false, "Allow only one hyperplane per constraint per iteration");

    env->settings->createSetting("Dual.ESH.Rootsearch.UseMaxFunction", false,
        "Perform rootsearch on max function, otherwise on individual constraints");

    // Dual strategy settings: Hyperplane generation

    env->settings->createSettingGroup("Dual", "HyperplaneCuts", "Generated hyperplane cuts",
        "These settings control how the cutting planes or supporting hyperplanes are generated.");

    env->settings->createSetting("Dual.HyperplaneCuts.ConstraintSelectionFactor", 0.5,
        "The fraction of violated constraints to generate supporting hyperplanes / cutting planes for", 0.0, 1.0);

    env->settings->createSetting("Dual.HyperplaneCuts.Delay", true, "Add hyperplane cuts to model only after optimal MIP solution");

    env->settings->createSetting("Dual.HyperplaneCuts.MaxConstraintFactor", 0.1,
        "Rootsearch performed on constraints with values larger than this factor times the maximum value", 1e-6, 1.0);

    env->settings->createSetting("Dual.HyperplaneCuts.MaxPerIteration", 200,
        "Maximal number of hyperplanes to add per iteration", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.HyperplaneCuts.UseIntegerCuts", false,
        "Add integer cuts for infeasible integer-combinations for binary problems");

    env->settings->createSetting("Dual.HyperplaneCuts.SaveHyperplanePoints", false,
        "Whether to save the points in the generated hyperplanes list", false);

    VectorString enumObjectiveRootsearch;
    enumObjectiveRootsearch.push_back("Always");
    enumObjectiveRootsearch.push_back("IfConvex");
    enumObjectiveRootsearch.push_back("Never");
    env->settings->createSetting("Dual.HyperplaneCuts.ObjectiveRootSearch",         static_cast<int>(ES_ObjectiveRootsearch::IfConvex), "When to use the objective root search",
        enumObjectiveRootsearch, 0);
    enumObjectiveRootsearch.clear();

    // TODO: activate
    // env->settings->createSetting(
    //    "HyperplaneCuts.UsePrimalObjectiveCut", "Dual", true, "Add an objective cut in the primal solution");

    // Dual strategy settings: MIP solver

    env->settings->createSettingGroup("Dual", "MIP", "MIP solver",
        "These settings control the general functionality of the MIP solver in the dual strategy. Note that "
        "solver-specific settings for Cplex, Gurobi and Cbc are available under the \"Subsolver\" category.");

    env->settings->createSetting("Dual.MIP.CutOff.InitialValue", SHOT_DBL_MAX, "Initial cutoff value to use", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.MIP.CutOff.UseInitialValue", false, "Use the initial cutoff value");

    env->settings->createSetting("Dual.MIP.CutOff.Tolerance", 0.00001,
        "An extra tolerance for the objective cutoff value (to prevent infeasible subproblems)", SHOT_DBL_MIN,
        SHOT_DBL_MAX);

    env->settings->createSetting("Dual.MIP.InfeasibilityRepair.IntegerCuts", true, "Allow feasibility repair of integer cuts");

    env->settings->createSetting("Dual.MIP.InfeasibilityRepair.IterationLimit", 100,
        "Max number of infeasible problems repaired without primal objective value improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.MIP.InfeasibilityRepair.TimeLimit", 10.0,
        "Time limit when reparing infeasible problem", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.MIP.InfeasibilityRepair.Use", true, "Enable the infeasibility repair strategy for nonconvex problems");

    env->settings->createSetting("Dual.MIP.OptimalityTolerance", 1e-6,
        "The reduced-cost tolerance for optimality in the MIP solver", 1e-9, 1e-2);

    env->settings->createSetting("Dual.MIP.NodeLimit", SHOT_DBL_MAX,
        "Node limit to use for MIP solver in single-tree strategy", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.MIP.NumberOfThreads", 0, "Number of threads to use in MIP solver: 0: Automatic", 0, 999);

    VectorString enumPresolve;
    enumPresolve.push_back("Never");
    enumPresolve.push_back("Once");
    enumPresolve.push_back("Always");
    env->settings->createSetting("Dual.MIP.Presolve.Frequency", static_cast<int>(ES_MIPPresolveStrategy::Once),
        "When to call the MIP presolve", enumPresolve, 0);
    enumPresolve.clear();

    env->settings->createSetting("Dual.MIP.Presolve.RemoveRedundantConstraints", false,
        "Remove redundant constraints (as determined by presolve)");

    env->settings->createSetting("Dual.MIP.Presolve.UpdateObtainedBounds", true, "Update bounds (from presolve) to the MIP model");

    env->settings->createSetting("Dual.MIP.SolutionLimit.ForceOptimal.Iteration", 10000,
        "Iterations without dual bound updates for forcing optimal MIP solution", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.MIP.SolutionLimit.ForceOptimal.Time", 1000.0,
        "Time (s) without dual bound updates for forcing optimal MIP solution", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.MIP.SolutionLimit.IncreaseIterations", 50,
        "Max number of iterations between MIP solution limit increases", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.MIP.SolutionLimit.Initial", 1, "Initial MIP solution limit", 1, SHOT_INT_MAX);

    env->settings->createSetting("Dual.MIP.SolutionLimit.UpdateTolerance", 0.001,
        "The constraint tolerance for when to update MIP solution limit", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.MIP.SolutionPool.Capacity", 100,
        "The maximum number of solutions in the solution pool", 0, SHOT_INT_MAX);

    VectorString enumMIPSolver;
    enumMIPSolver.push_back("Cplex");
    enumMIPSolver.push_back("Gurobi");
    enumMIPSolver.push_back("Cbc");
    enumMIPSolver.push_back("Highs");

    ES_MIPSolver usedMIPSolver;

#ifdef HAS_GUROBI
    usedMIPSolver = ES_MIPSolver::Gurobi;
#elif HAS_CPLEX
    usedMIPSolver = ES_MIPSolver::Cplex;
#elif HAS_CBC
    usedMIPSolver = ES_MIPSolver::Cbc;
#elif HAS_HIGHS
    usedMIPSolver = ES_MIPSolver::Highs;
#else
    env->output->outputCritical(" SHOT has not been compiled with support for any MIP solver.");
#endif

    env->settings->createSetting("Dual.MIP.Solver", static_cast<int>(usedMIPSolver), "Which MIP solver to use", enumMIPSolver, 0);
    enumMIPSolver.clear();

    env->settings->createSetting("Dual.MIP.UpdateObjectiveBounds", false, "Update nonlinear objective variable bounds to primal/dual bounds");

    // Convex bounding

    env->settings->createSettingGroup("Dual", "ConvexBounding", "Convex Bounding",
        "These settings control the convex bounding strategy that solves a MIP problem with all hyperplane cuts "
        "generated for convex constraints so far and ignoring those generated for nonconvex constraints. This will "
        "give a dual bound for the nonconvex problem.");

    env->settings->createSetting("Dual.ConvexBounding.Use", true, "Enable the convex bounding strategy for nonconvex problems");

    env->settings->createSetting("Dual.ConvexBounding.IdleIterations", 10,
        "How often the convex bounding strategy should be executed. 0 = Every iteration with generated hyperplanes for "
        "nonconvexities, 1 = Every second iteration, etc.",
        0, SHOT_INT_MAX);

    // Primal settings: reduction cuts for nonconvex problems

    env->settings->createSettingGroup("Dual", "ReductionCut", "Dual reduction cut",
        "These settings control the added dual reduction cuts from the primal solution that will try to force a better "
        "primal solution. This functionality is only used if SHOT cannot deduce that the problem is convex .");

    env->settings->createSetting("Dual.ReductionCut.Use", true, "Enable the dual reduction cut strategy for nonconvex problems");

    VectorString enumReductionCutStrategy;
    enumReductionCutStrategy.push_back("Fraction");
    enumReductionCutStrategy.push_back("GoldenRatio");

    ES_ReductionCutStrategy reductionCutStrategy;

    reductionCutStrategy = ES_ReductionCutStrategy::Fraction;

    env->settings->createSetting("Dual.ReductionCut.Strategy", static_cast<int>(reductionCutStrategy),
        "The reduction cut strategy to use", enumReductionCutStrategy,
        static_cast<int>(ES_ReductionCutStrategy::Fraction));
    enumReductionCutStrategy.clear();

    env->settings->createSetting("Dual.ReductionCut.MaxIterations", 20,
        "Max number of primal cut reduction without primal improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.ReductionCut.ReductionFactor", 0.001, "The factor used to reduce the cutoff value", 0, 1.0);

    // Dual strategy settings: Relaxation strategies

    env->settings->createSettingGroup("Dual", "Relaxation", "Relaxation strategies",
        "These settings contorl various aspects regarding integer-relaxation of the dual problem.");

    env->settings->createSetting("Dual.Relaxation.Use", true, "Initially solve continuous dual relaxations");

    env->settings->createSetting("Dual.Relaxation.Frequency", 0, "The frequency to solve an LP problem: 0: Disable", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.Relaxation.IterationLimit", 200,
        "The max number of relaxed LP problems to solve initially", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.Relaxation.MaxLazyConstraints", 0,
        "Max number of lazy constraints to add in relaxed solutions in single-tree strategy", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "Dual.Relaxation.TerminationTolerance", 0.5, "Time limit (s) when solving LP problems initially");

    env->settings->createSetting(
        "Dual.Relaxation.TimeLimit", 30.0, "Time limit (s) when solving LP problems initially", 0, SHOT_DBL_MAX);

    // Dual strategy settings: Main tree strategy

    env->settings->createSettingGroup("Dual", "TreeStrategy", "Tree strategy",
        "The single-tree strategy is normally more efficient than the multi-tree one. However, not all MIP solvers "
        "support the required lazy constraint callbacks. These settings selects this strategy and controls its "
        "behaviour.");

    VectorString enumSolutionStrategy;
    enumSolutionStrategy.push_back("Multi-tree");
    enumSolutionStrategy.push_back("Single-tree");
    env->settings->createSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree),
        "The main strategy to use", enumSolutionStrategy, 0);
    enumSolutionStrategy.clear();

    env->settings->createSetting("Dual.TreeStrategy.Multi.Reinitialize", false,
        "Reinitialize the dual model in the subsolver each iteration", true);

    // Optimization model settings

    env->settings->createSettingGroup("Model", "", "Optimization model",
        "These settings control various aspects of SHOT's representation  for and handling of the provided "
        "optimization model.");

    // Bound tightening

    env->settings->createSettingGroup("Model", "BoundTightening", "Bound tightening",
        "SHOT performs bound tightening to strengthen the internal representation of the problem. These settings "
        "control how and when bound tightening is performed.");

    // Bound tightening: feasibility based

    env->settings->createSetting("Model.BoundTightening.FeasibilityBased.MaxIterations", 5, "Maximal number of bound tightening iterations");

    env->settings->createSetting("Model.BoundTightening.FeasibilityBased.TimeLimit", 2.0,
        "Time limit for bound tightening", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Model.BoundTightening.FeasibilityBased.Use", true, "Peform feasibility-based bound tightening");

    env->settings->createSetting("Model.BoundTightening.FeasibilityBased.UseNonlinear", true,
        "Peform feasibility-based bound tightening on nonlinear expressions");

    // Bound tightening: initial POA

    env->settings->createSetting("Model.BoundTightening.InitialPOA.ConstraintTolerance", 1e-1, "Constraint termination tolerance");

    VectorString enumCutStrategy;
    enumCutStrategy.push_back("ESH");
    enumCutStrategy.push_back("ECP");
    env->settings->createSetting("Model.BoundTightening.InitialPOA.CutStrategy",         static_cast<int>(ES_HyperplaneCutStrategy::ECP), "Dual cut strategy", enumCutStrategy, 0);
    enumCutStrategy.clear();

    env->settings->createSetting("Model.BoundTightening.InitialPOA.IterationLimit", 50, "Iteration limit for POA");

    env->settings->createSetting("Model.BoundTightening.InitialPOA.ObjectiveConstraintTolerance", 1e-3,
        "Objective constraint termination tolerance");

    env->settings->createSetting("Model.BoundTightening.InitialPOA.ObjectiveGapAbsolute", 1e-1, "Absolute objective gap termination level");

    env->settings->createSetting("Model.BoundTightening.InitialPOA.ObjectiveGapRelative", 1e-1, "Relative objective gap termination level");

    env->settings->createSetting("Model.BoundTightening.InitialPOA.StagnationConstraintTolerance", 1e-2,
        "Tolerance factor for when no progress is made");

    env->settings->createSetting("Model.BoundTightening.InitialPOA.StagnationIterationLimit", 5,
        "Limit for iterations without significant progress");

    env->settings->createSetting("Model.BoundTightening.InitialPOA.Use", false, "Create an initial polyhedral outer approximation");

    env->settings->createSetting("Model.BoundTightening.InitialPOA.TimeLimit", 5.0, "Time limit for initial POA");

    // Convexity settings

    env->settings->createSettingGroup(
        "Model", "Convexity", "Convexity", "These settings control the convexity detection functionality");

    env->settings->createSetting("Model.Convexity.AssumeConvex", false, "Assume that the problem is convex");

    env->settings->createSetting("Model.Convexity.Quadratics.EigenValueTolerance", 1e-5,
        "Convexity tolerance for the eigenvalues of the Hessian matrix for quadratic terms", 0.0, SHOT_DBL_MAX);

    // Variable settings

    env->settings->createSettingGroup("Model", "Variables", "Variables",
        "These settings control the maximum variable bounds allowed in SHOT. Projection will be performed onto these "
        "intervals. Note that the MIP solvers may have stricter requirements, in which case those may be used.");

    env->settings->createSetting("Model.Variables.Continuous.MinimumLowerBound", -1e50,
        "Minimum lower bound for continuous variables", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSetting("Model.Variables.Continuous.MaximumUpperBound", 1e50,
        "Maximum upper bound for continuous variables", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSetting("Model.Variables.Integer.MinimumLowerBound", -2.0e9,
        "Minimum lower bound for integer variables", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSetting("Model.Variables.Integer.MaximumUpperBound", 2.0e9,
        "Maximum upper bound for integer variables", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSetting("Model.Variables.NonlinearObjectiveVariable.Bound", 1e12,
        "Max absolute bound for the auxiliary nonlinear objective variable", SHOT_DBL_MIN, SHOT_DBL_MAX);

    // Reformulation settings

    env->settings->createSettingGroup("Model", "Reformulation", "Automatic reformulations",
        "These settings control the automatic reformulations performed in SHOT.");

    // Reformulations for bilinears
    env->settings->createSetting("Model.Reformulation.Bilinear.AddConvexEnvelope", false,
        "Add convex envelopes (subject to original bounds) to bilinear terms");

    // Reformulations for integer bilinears
    VectorString enumBilinearIntegerReformulation;
    enumBilinearIntegerReformulation.push_back("No");
    enumBilinearIntegerReformulation.push_back("No if nonconvex quadratic terms allowed by MIP solver");
    enumBilinearIntegerReformulation.push_back("Yes");
    env->settings->createSetting("Model.Reformulation.Bilinear.IntegerFormulation",         static_cast<int>(ES_ReformulateBilinearInteger::NoIfQuadraticSupport), "Reformulate integer bilinear terms",
        enumBilinearIntegerReformulation, 0);
    enumBilinearIntegerReformulation.clear();

    env->settings->createSetting("Model.Reformulation.Bilinear.IntegerFormulation.MaxDomain", 100,
        "Do not reformulate integer variables in bilinear terms which can assume more than this number of discrete "
        "values",
        2, SHOT_INT_MAX);

    // Reformulations for constraints
    VectorString enumNonlinearTermPartitioning;
    enumNonlinearTermPartitioning.push_back("Always");
    enumNonlinearTermPartitioning.push_back("If result is convex");
    enumNonlinearTermPartitioning.push_back("Never");
    env->settings->createSetting("Model.Reformulation.Constraint.PartitionNonlinearTerms",         static_cast<int>(ES_PartitionNonlinearSums::IfConvex), "When to partition nonlinear sums in constraints",
        enumNonlinearTermPartitioning, 0);

    env->settings->createSetting("Model.Reformulation.Constraint.PartitionQuadraticTerms",         static_cast<int>(ES_PartitionNonlinearSums::IfConvex), "When to partition quadratic sums in constraints",
        enumNonlinearTermPartitioning, 0);

    // Reformulations for monomials

    env->settings->createSetting("Model.Reformulation.Monomials.Extract", true, "Extract monomial terms from nonlinear expressions");

    VectorString enumBinaryMonomialReformulation;
    enumBinaryMonomialReformulation.push_back("None");
    enumBinaryMonomialReformulation.push_back("Simple");
    enumBinaryMonomialReformulation.push_back("Costa and Liberti");
    env->settings->createSetting("Model.Reformulation.Monomials.Formulation",         static_cast<int>(ES_ReformulationBinaryMonomials::Simple), "How to reformulate binary monomials",
        enumBinaryMonomialReformulation, 0);
    enumBinaryMonomialReformulation.clear();

    // Reformulations for objective functions
    env->settings->createSetting("Model.Reformulation.ObjectiveFunction.Epigraph.Use", false,
        "Reformulates a nonlinear objective as an auxiliary constraint");

    env->settings->createSetting("Model.Reformulation.ObjectiveFunction.PartitionNonlinearTerms",         static_cast<int>(ES_PartitionNonlinearSums::IfConvex), "When to partition nonlinear sums in objective function",
        enumNonlinearTermPartitioning, 0);

    env->settings->createSetting("Model.Reformulation.ObjectiveFunction.PartitionQuadraticTerms",         static_cast<int>(ES_PartitionNonlinearSums::IfConvex), "When to partition quadratic sums in objective function",
        enumNonlinearTermPartitioning, 0);

    enumNonlinearTermPartitioning.clear();

    // Reformulations for signomials

    env->settings->createSetting("Model.Reformulation.Signomials.Extract", true, "Extract signomial terms from nonlinear expressions");

    // Reformulations for quadratic objective and constraints

    VectorString enumQuadExtractStrategy;
    enumQuadExtractStrategy.push_back("Do not extract");
    enumQuadExtractStrategy.push_back("Extract to same objective or constraint");
    enumQuadExtractStrategy.push_back("Extract to quadratic equality constraint if nonconvex");
    enumQuadExtractStrategy.push_back("Extract to quadratic equality constraint even if convex");

    env->settings->createSetting("Model.Reformulation.Quadratics.ExtractStrategy",         static_cast<int>(ES_QuadraticTermsExtractStrategy::ExtractTermsToSame),
        "How to extract quadratic terms from nonlinear expressions", enumQuadExtractStrategy, 0);
    enumQuadExtractStrategy.clear();

    VectorString enumQPStrategy;
    enumQPStrategy.push_back("All nonlinear");
    enumQPStrategy.push_back("Use quadratic objective");
    enumQPStrategy.push_back("Use convex quadratic objective and constraints");
    enumQPStrategy.push_back("Use nonconvex quadratic objective and constraints");

    env->settings->createSetting("Model.Reformulation.Quadratics.Strategy",         static_cast<int>(ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained),
        "How to treat quadratic functions", enumQPStrategy, 0);
    enumQPStrategy.clear();

    VectorString enumQPDecomposition;
    enumQPDecomposition.push_back("No decomposition");
    enumQPDecomposition.push_back("Eigenvalue decomposition");
    enumQPDecomposition.push_back("LDL decomposition");

    env->settings->createSetting("Model.Reformulation.Quadratics.Decomposition.Method",         static_cast<int>(ES_QuadraticDecomposition::LDLDecomposition),
        "Whether to use the eigenvalue decomposition of convex quadratic functions", enumQPDecomposition, 0);
    enumQPDecomposition.clear();

    VectorString enumQuadraticDecompCoeffStrategy;
    enumQuadraticDecompCoeffStrategy.push_back("Coefficient is included in reformulation");
    enumQuadraticDecompCoeffStrategy.push_back("Coefficient remains");

    env->settings->createSetting("Model.Reformulation.Quadratics.Decomposition.Formulation",         static_cast<int>(ES_QuadraticDecompositionFormulation::CoefficientReformulated),
        "Placement of the original term cofficient when decomposing a quadratic term", enumQuadraticDecompCoeffStrategy,
        0);
    enumQuadraticDecompCoeffStrategy.clear();

    env->settings->createSetting("Model.Reformulation.Quadratics.Decomposition.Tolerance", 1e-7,
        "Terms with corresponding eigenvalues or diagonal elements in D-matrix smaller than this value will be ignored",
        0.0, SHOT_DBL_MAX);

    // Modeling system settings

    env->settings->createSettingGroup("ModelingSystem", "", "Modeling system",
        "These settings control functionality used in the interfaces to different modeling environments.");

    // Logging and output settings

    env->settings->createSettingGroup("Output", "", "Solver output",
        "These settings control how much and what output is shown to the user from the solver.");

    env->settings->createSetting("Output.Console.DualSolver.Show", false, "Show output from dual solver on console");

    VectorString enumIterationDetail;
    enumIterationDetail.push_back("Full");
    enumIterationDetail.push_back("On objective gap update");
    enumIterationDetail.push_back("On objective gap update and all primal NLP calls");
    env->settings->createSetting("Output.Console.Iteration.Detail",         static_cast<int>(ES_IterationOutputDetail::ObjectiveGapUpdates), "When should the fixed strategy be used",
        enumIterationDetail, 0);
    enumIterationDetail.clear();

    VectorString enumLogLevel;
    enumLogLevel.push_back("Trace");
    enumLogLevel.push_back("Debug");
    enumLogLevel.push_back("Info");
    enumLogLevel.push_back("Warning");
    enumLogLevel.push_back("Error");
    enumLogLevel.push_back("Critical");
    enumLogLevel.push_back("Off");
    env->settings->createSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info),
        "Log level for console output", enumLogLevel, 0);

    env->settings->createSetting("Output.Console.PrimalSolver.Show", false, "Show output from primal solver on console");

    env->settings->createSetting("Output.Debug.Enable", false, "Use debug functionality");

    env->settings->createSetting("Output.Debug.Path", empty, "The folder where to save the debug information", false);

    env->settings->createSetting("Output.File.LogLevel", static_cast<int>(E_LogLevel::Info), "Log level for file output", enumLogLevel, 0);
    enumLogLevel.clear();

    env->settings->createSetting("Output.GAMS.AlternateSolutionsFile", std::string(),
        "Name of GAMS GDX file to write alternative solutions to", false);

    VectorString enumOutputDirectory;
    enumOutputDirectory.push_back("Problem directory");
    enumOutputDirectory.push_back("Program directory");
    env->settings->createSetting("Output.OutputDirectory", static_cast<int>(ES_OutputDirectory::Program),
        "Where to save the output files", enumOutputDirectory, 0);
    enumOutputDirectory.clear();

    env->settings->createSetting("Output.SaveNumberOfSolutions", 1, "Save max this number of primal solutions to OSrL or GDX file");

    env->settings->createSettingGroup(
        "Primal", "", "Primal heuristics", "These settings control the primal heuristics used in SHOT.");

    env->settings->createSettingGroup("Primal", "FixedInteger", "Fixed-integer (NLP) strategy",
        "The main primal strategy in SHOT is to solve integer-fixed NLP problems. These settings control, e.g., how "
        "often NLP problems are solved.");

    // Primal settings: Fixed integer strategy
    VectorString enumPrimalNLPStrategy;
    enumPrimalNLPStrategy.push_back("Use each iteration");
    enumPrimalNLPStrategy.push_back("Based on iteration or time");
    enumPrimalNLPStrategy.push_back("Based on iteration or time, and for all feasible MIP solutions");

    env->settings->createSetting("Primal.FixedInteger.CallStrategy",         static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions),
        "When should the fixed strategy be used", enumPrimalNLPStrategy, 0);
    enumPrimalNLPStrategy.clear();

    env->settings->createSetting("Primal.FixedInteger.CreateInfeasibilityCut", false, "Create a cut from an infeasible solution point");

    env->settings->createSetting("Primal.FixedInteger.Frequency.Dynamic", true, "Dynamically update the call frequency based on success");

    env->settings->createSetting("Primal.FixedInteger.Frequency.Iteration", 10, "Max number of iterations between calls", 0, SHOT_INT_MAX);

    env->settings->createSetting("Primal.FixedInteger.Frequency.Time", 5.0, "Max duration (s) between calls", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Primal.FixedInteger.DualPointGap.Relative", 0.001,
        "If the objective gap between the MIP point and dual solution is less than this the fixed strategy is "
        "activated",
        0, SHOT_DBL_MAX);

    env->settings->createSetting("Primal.FixedInteger.IterationLimit", 10000000, "Max number of iterations per call", 0, SHOT_INT_MAX);

    env->settings->createSetting("Primal.FixedInteger.OnlyUniqueIntegerCombinations", true,
        "Whether to resolve with the same integer combination, e.g. for nonconvex problems with different continuous "
        "variable starting points");

    VectorString enumPrimalNLPSolver;
    enumPrimalNLPSolver.push_back("Ipopt");
    enumPrimalNLPSolver.push_back("GAMS");
    enumPrimalNLPSolver.push_back("SHOT");

    env->settings->createSetting("Primal.FixedInteger.Solver", static_cast<int>(ES_PrimalNLPSolver::Ipopt),
        "NLP solver to use", enumPrimalNLPSolver, 0);
    enumPrimalNLPSolver.clear();

    VectorString enumPrimalBoundNLPStartingPoint;
    enumPrimalBoundNLPStartingPoint.push_back("All");
    enumPrimalBoundNLPStartingPoint.push_back("First");
    enumPrimalBoundNLPStartingPoint.push_back("All feasible");
    enumPrimalBoundNLPStartingPoint.push_back("First and all feasible");
    enumPrimalBoundNLPStartingPoint.push_back("With smallest constraint deviation");
    env->settings->createSetting("Primal.FixedInteger.Source",         static_cast<int>(ES_PrimalNLPFixedPoint::FirstAndFeasibleSolutions), "Source of fixed MIP solution point",
        enumPrimalBoundNLPStartingPoint, 0);
    enumPrimalBoundNLPStartingPoint.clear();

    VectorString enumPrimalBoundNLPProblemSource;
    enumPrimalBoundNLPProblemSource.push_back("Original problem");
    enumPrimalBoundNLPProblemSource.push_back("Reformulated problem");
    enumPrimalBoundNLPProblemSource.push_back("Both");
    env->settings->createSetting("Primal.FixedInteger.SourceProblem",         static_cast<int>(ES_PrimalNLPProblemSource::OriginalProblem),
        "Which problem formulation to use for NLP problem", enumPrimalBoundNLPProblemSource, 0);
    enumPrimalBoundNLPProblemSource.clear();

    env->settings->createSetting("Primal.FixedInteger.TimeLimit", 10.0, "Time limit (s) per NLP problem", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Primal.FixedInteger.Use", true, "Use the fixed integer primal strategy");

    env->settings->createSetting("Primal.FixedInteger.UsePresolveBounds", false,
        "Use variable bounds from MIP in NLP problems. Warning! Does not seem to work", true);

    env->settings->createSetting("Primal.FixedInteger.Warmstart", true, "Warm start the NLP solver");

    // Primal settings: rootsearch

    env->settings->createSettingGroup("Primal", "Rootsearch", "Primal root search",
        "SHOT can utilize root searches between the dual solution point and an integer-fixed interior point. This "
        "setting controls whether this strategy is used.");

    env->settings->createSetting("Primal.Rootsearch.Use", true, "Use a rootsearch to find primal solutions");

    // Primal settings: tolerances for accepting primal solutions

    env->settings->createSettingGroup("Primal", "Tolerances", "Primal solution tolerances",
        "These settings sets various tolerances for accepting primal solutions.");

    env->settings->createSetting("Primal.Tolerance.TrustLinearConstraintValues", true,
        "Trust that subsolvers (NLP, MIP) give primal solutions that respect linear constraints");

    env->settings->createSetting("Primal.Tolerance.Integer", 1e-5, "Integer tolerance for accepting primal solutions");

    env->settings->createSetting("Primal.Tolerance.LinearConstraint", 1e-6, "Linear constraint tolerance for accepting primal solutions");

    env->settings->createSetting("Primal.Tolerance.NonlinearConstraint", 1e-5,
        "Nonlinear constraint tolerance for accepting primal solutions");

    // Strategy settings

    env->settings->createSettingGroup("Strategy", "", "Strategy", "Overall strategy parameters used in SHOT.");

    env->settings->createSetting("Strategy.UseRecommendedSettings", true,
        "Modifies some settings to their recommended values based on the strategy");

    // Subsolver settings: Cplex

    env->settings->createSettingGroup("Subsolver", "", "Subsolver functionality",
        "These settings allow for more direct control of the  different subsolvers utilized in SHOT.");

#ifdef HAS_CPLEX

    env->settings->createSettingGroup("Subsolver", "Cplex", "Cplex", "");

    env->settings->createSetting("Subsolver.Cplex.AddRelaxedLazyConstraintsAsLocal", false,
        "Whether to add lazy constraints generated in relaxed points as local or global");

    VectorString enumCplexOptimalityTarget;
    enumCplexOptimalityTarget.push_back("Automatic");
    enumCplexOptimalityTarget.push_back("Searches for a globally optimal solution to a convex model");
    enumCplexOptimalityTarget.push_back("Searches for a solution that satisfies first-order optimality conditions, but "
                                        "is not necessarily globally optimal");
    enumCplexOptimalityTarget.push_back("Searches for a globally "
                                        "optimal solution to a nonconvex model");
    env->settings->createSetting("Subsolver.Cplex.OptimalityTarget", 0,
        "Specifies how CPLEX treats nonconvex quadratics", enumCplexOptimalityTarget, 0);
    enumCplexOptimalityTarget.clear();

    VectorString enumCplexFeasOptMode;
    enumCplexFeasOptMode.push_back("Minimize the sum of all required relaxations in first phase only");
    enumCplexFeasOptMode.push_back("Minimize the sum of all required relaxations in first phase and execute second "
                                   "phase to find optimum among minimal relaxations");
    enumCplexFeasOptMode.push_back("Minimize the number of constraints and bounds requiring relaxation in first "
                                   "phase only");
    enumCplexFeasOptMode.push_back("Minimize the sum of squares of required "
                                   "relaxations in first phase only");
    enumCplexFeasOptMode.push_back(" Minimize the sum of squares of required relaxations in first phase and execute "
                                   "second phase to find optimum among minimal relaxations");
    env->settings->createSetting("Subsolver.Cplex.FeasOptMode", 0, "Strategy to use for the feasibility repair", enumCplexFeasOptMode, 0);
    enumCplexFeasOptMode.clear();

    VectorString enumCplexMIPEmphasis;
    enumCplexMIPEmphasis.push_back("Balanced");
    enumCplexMIPEmphasis.push_back("Feasibility");
    enumCplexMIPEmphasis.push_back("Optimality");
    enumCplexMIPEmphasis.push_back("Best bound");
    enumCplexMIPEmphasis.push_back("Hidden feasible");
    env->settings->createSetting("Subsolver.Cplex.MIPEmphasis", 1, "Sets the MIP emphasis", enumCplexMIPEmphasis, 0);
    enumCplexMIPEmphasis.clear();

    env->settings->createSetting("Subsolver.Cplex.MemoryEmphasis", 0, "Try to conserve memory when possible", 0, 1);

    VectorString enumCplexNodeFile;
    enumCplexNodeFile.push_back("No file");
    enumCplexNodeFile.push_back("Compressed in memory");
    enumCplexNodeFile.push_back("On disk");
    enumCplexNodeFile.push_back("Compressed on disk");
    env->settings->createSetting(
        "Subsolver.Cplex.NodeFile", 1, "Where to store the node file", enumCplexNodeFile, 0);
    enumCplexNodeFile.clear();

    env->settings->createSetting("Subsolver.Cplex.NumericalEmphasis", 1, "Emphasis on numerical stability", 0, 1);

    VectorString enumCplexParallelMode;
    enumCplexParallelMode.push_back("Opportunistic");
    enumCplexParallelMode.push_back("Automatic");
    enumCplexParallelMode.push_back("Deterministic");
    env->settings->createSetting("Subsolver.Cplex.ParallelMode", 0,
        "Controls how much time and memory should be used when filling the solution pool", enumCplexParallelMode, -1);
    enumCplexParallelMode.clear();

    VectorString enumCplexProbe;
    enumCplexProbe.push_back("No probing");
    enumCplexProbe.push_back("Automatic");
    enumCplexProbe.push_back("Moderate");
    enumCplexProbe.push_back("Aggressive");
    enumCplexProbe.push_back("Very aggressive");
    env->settings->createSetting("Subsolver.Cplex.Probe", 0, "Sets the MIP probing level", enumCplexProbe, -1);
    enumCplexProbe.clear();

    env->settings->createSetting("Subsolver.Cplex.SolutionPoolGap", 1.0e+75,
        "Sets the relative gap filter on objective values in the solution pool", 0, 1.0e+75);

    VectorString enumCplexSolPoolIntensity;
    enumCplexSolPoolIntensity.push_back("Automatic");
    enumCplexSolPoolIntensity.push_back("Mild");
    enumCplexSolPoolIntensity.push_back("Moderate");
    enumCplexSolPoolIntensity.push_back("Aggressive");
    enumCplexSolPoolIntensity.push_back("Very aggressive");
    env->settings->createSetting("Subsolver.Cplex.SolutionPoolIntensity", 0,
        "Controls how much time and memory should be used when filling the solution pool", enumCplexSolPoolIntensity,
        0);
    enumCplexSolPoolIntensity.clear();

    VectorString enumCplexSolPoolReplace;
    enumCplexSolPoolReplace.push_back("Replace oldest");
    enumCplexSolPoolReplace.push_back("Replace worst");
    enumCplexSolPoolReplace.push_back("Find diverse");
    env->settings->createSetting("Subsolver.Cplex.SolutionPoolReplace", 0,
        "How to replace solutions in the solution pool when full", enumCplexSolPoolReplace, 0);
    enumCplexSolPoolReplace.clear();

    env->settings->createSetting("Subsolver.Cplex.UseGenericCallback", false, "Use the new generic callback in the single-tree strategy");

    std::string workdir = "";
    env->settings->createSetting("Subsolver.Cplex.WorkDirectory", workdir, "Directory for swap file");

    env->settings->createSetting("Subsolver.Cplex.WorkMemory", 0.0, "Memory limit for when to start swapping to disk", 0.0, 1.0e+75);

#endif

    // Subsolver settings: Gurobi

#ifdef HAS_GUROBI

    env->settings->createSettingGroup("Subsolver", "Gurobi", "Gurobi", "");

    env->settings->createSetting("Subsolver.Gurobi.Heuristics", 0.05, "The relative amount of time spent in MIP heuristics.", 0.0, 1.0);

    VectorString enumGurobiMIPFocus;
    enumGurobiMIPFocus.push_back("Automatic");
    enumGurobiMIPFocus.push_back("Feasibility");
    enumGurobiMIPFocus.push_back("Optimality");
    enumGurobiMIPFocus.push_back("Best bound");
    env->settings->createSetting("Subsolver.Gurobi.MIPFocus", 0, "MIP focus", enumGurobiMIPFocus, 0);
    enumGurobiMIPFocus.clear();

    VectorString enumGurobiNumericFocus;
    enumGurobiNumericFocus.push_back("Automatic");
    enumGurobiNumericFocus.push_back("Mild");
    enumGurobiNumericFocus.push_back("Moderate");
    enumGurobiNumericFocus.push_back("Aggressive");
    env->settings->createSetting("Subsolver.Gurobi.NumericFocus", 1, "MIP focus", enumGurobiNumericFocus, 0);
    enumGurobiNumericFocus.clear();

    VectorString enumGurobiPoolSearchMode;
    enumGurobiPoolSearchMode.push_back("No extra effort");
    enumGurobiPoolSearchMode.push_back("Try to find solutions");
    enumGurobiPoolSearchMode.push_back("Find n best solutions");
    env->settings->createSetting("Subsolver.Gurobi.PoolSearchMode", 0, "Finds extra solutions", enumGurobiPoolSearchMode, 0);
    enumGurobiPoolSearchMode.clear();

    env->settings->createSetting("Subsolver.Gurobi.PoolSolutions", 10, "Determines how many MIP solutions are stored", 1, 2000000000);

    VectorString enumGurobiScaleFlag;
    enumGurobiScaleFlag.push_back("Automatic");
    enumGurobiScaleFlag.push_back("Off");
    enumGurobiScaleFlag.push_back("Mild");
    enumGurobiScaleFlag.push_back("Moderate");
    enumGurobiScaleFlag.push_back("Aggressive");
    env->settings->createSetting("Subsolver.Gurobi.ScaleFlag", -1, "Controls model scaling", enumGurobiScaleFlag, -1);
    enumGurobiScaleFlag.clear();

#endif

    // Subsolver settings: Cbc

#ifdef HAS_CBC

    env->settings->createSettingGroup("Subsolver", "Cbc", "Cbc", "");

    env->settings->createSetting("Subsolver.Cbc.AutoScale", false,
        "Whether to scale objective, rhs and bounds of problem if they look odd (experimental)");

    VectorString enumCbcNodeStrategy;
    enumCbcNodeStrategy.push_back("depth");
    enumCbcNodeStrategy.push_back("downdepth");
    enumCbcNodeStrategy.push_back("downfewest");
    enumCbcNodeStrategy.push_back("fewest");
    enumCbcNodeStrategy.push_back("hybrid");
    enumCbcNodeStrategy.push_back("updepth");
    enumCbcNodeStrategy.push_back("upfewest");
    env->settings->createSetting("Subsolver.Cbc.NodeStrategy", 4, "Node strategy", enumCbcNodeStrategy, 0);
    enumCbcNodeStrategy.clear();

    env->settings->createSetting("Subsolver.Cbc.DeterministicParallelMode", false, "Run Cbc with multiple threads in deterministic mode");

    VectorString enumCbcScaling;
    enumCbcScaling.push_back("automatic");
    enumCbcScaling.push_back("dynamic");
    enumCbcScaling.push_back("equilibrium");
    enumCbcScaling.push_back("geometric");
    enumCbcScaling.push_back("off");
    enumCbcScaling.push_back("rowsonly");
    env->settings->createSetting("Subsolver.Cbc.Scaling", 4, "Whether to scale problem", enumCbcScaling, 0);
    enumCbcScaling.clear();

    VectorString enumStrategy;
    enumStrategy.push_back("easy problems");
    enumStrategy.push_back("default");
    enumStrategy.push_back("aggressive");
    env->settings->createSetting("Subsolver.Cbc.Strategy", 1, "This turns on newer features", enumStrategy, 0);
    enumStrategy.clear();

#endif

#ifdef HAS_HIGHS

    env->settings->createSettingGroup("Subsolver", "Highs", "Highs", "");

    env->settings->createSetting("Subsolver.Highs.MIPAllowRestart", true, "Whether MIP restart is permitted");

    env->settings->createSetting("Subsolver.Highs.MIPDetectSymmetry", true, "Whether MIP symmetry should be detected");

    env->settings->createSetting("Subsolver.Highs.MIPHeuristicEffort", 0.05, "Effort spent for MIP heuristics", 0, 1);

    env->settings->createSetting("Subsolver.Highs.MIPHeuristicRunZiRound", false, "Run Zi Round heuristic");

    env->settings->createSetting("Subsolver.Highs.MIPHeuristicRunShifting", false, "Run Shifting heuristic");

    VectorString enumHighsRunCrossover;
    enumHighsRunCrossover.push_back("off");
    enumHighsRunCrossover.push_back("choose");
    enumHighsRunCrossover.push_back("on");
    env->settings->createSetting("Subsolver.Highs.RunCrossover", 0, "Run crossover", enumHighsRunCrossover, 0);
    enumHighsRunCrossover.clear();

    VectorString enumHighsDebugLevel;
    enumHighsDebugLevel.push_back("off");
    enumHighsDebugLevel.push_back("low");
    enumHighsDebugLevel.push_back("medium");
    enumHighsDebugLevel.push_back("high");
    env->settings->createSetting("Subsolver.Highs.DebugLevel", 0, "Debug level for HiGHS internal assertions", enumHighsDebugLevel, 0);
    enumHighsDebugLevel.clear();

    /*
    VectorString enumHighsMIPIPMSolver;
    enumHighsMIPIPMSolver.push_back("choose");
    enumHighsMIPIPMSolver.push_back("ipx");
    enumHighsMIPIPMSolver.push_back("hipo");
    env->settings->createSetting("Subsolver.Highs.MIPIPMSolver", 1, "MIP IPM solver", enumHighsMIPIPMSolver, 0);
    enumHighsMIPIPMSolver.clear();
    */

    VectorString enumHighsParallel;
    enumHighsParallel.push_back("off");
    enumHighsParallel.push_back("choose");
    enumHighsParallel.push_back("on");
    env->settings->createSetting("Subsolver.Highs.Parallel", 1, "Use parallelization", enumHighsParallel, 0);
    enumHighsParallel.clear();

    VectorString enumHighsPresolve;
    enumHighsPresolve.push_back("off");
    enumHighsPresolve.push_back("choose");
    enumHighsPresolve.push_back("on");
    env->settings->createSetting("Subsolver.Highs.Presolve", 1, "Use presolve", enumHighsPresolve, 0);
    enumHighsPresolve.clear();

#endif

    // Subsolver settings: Ipopt

#ifdef HAS_IPOPT

    env->settings->createSettingGroup("Subsolver", "Ipopt", "Ipopt", "");

    env->settings->createSetting("Subsolver.Ipopt.ConstraintViolationTolerance", 1E-8,
        "Constraint violation tolerance in Ipopt", SHOT_DBL_MIN, SHOT_DBL_MAX);

    VectorString enumIPOptSolver;
    enumIPOptSolver.push_back("Default");
    enumIPOptSolver.push_back("MA27");
    enumIPOptSolver.push_back("MA57");
    enumIPOptSolver.push_back("MA86");
    enumIPOptSolver.push_back("MA97");
    enumIPOptSolver.push_back("MUMPS");
    env->settings->createSetting("Subsolver.Ipopt.LinearSolver", static_cast<int>(ES_IpoptSolver::IpoptDefault),
        "Ipopt linear subsolver", enumIPOptSolver, 0);
    enumIPOptSolver.clear();

    env->settings->createSetting("Subsolver.Ipopt.MaxIterations", 1000, "Maximum number of iterations");

    env->settings->createSetting("Subsolver.Ipopt.RelativeConvergenceTolerance", 1E-8, "Relative convergence tolerance");

#endif

    env->settings->createSettingGroup("Subsolver", "SHOT", "SHOT primal NLP solver", "");

    env->settings->createSetting("Subsolver.SHOT.ReuseHyperplanes.Use", true, "Reuse valid generated hyperplanes in main dual model.");
    env->settings->createSetting("Subsolver.SHOT.ReuseHyperplanes.Fraction", 0.1,
        "The fraction of generated hyperplanes to reuse.", 0.0, 1.0);

    env->settings->createSetting("Subsolver.SHOT.UseFBBT", true, "Do FBBT on NLP problem.");

    // Subsolver settings: root searches

    env->settings->createSettingGroup(
        "Subsolver", "Rootsearch", "Root search solver", "Settings for the Boost rootsearch functionality.");

    env->settings->createSetting("Subsolver.Rootsearch.ActiveConstraintTolerance", 0.0,
        "Epsilon constraint tolerance for root search", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Subsolver.Rootsearch.MaxIterations", 100, "Maximal root search iterations", 0, SHOT_INT_MAX);

    VectorString enumRootsearchMethod;
    enumRootsearchMethod.push_back("TOMS748");
    enumRootsearchMethod.push_back("Bisection");
    env->settings->createSetting("Subsolver.Rootsearch.Method", static_cast<int>(ES_RootsearchMethod::BoostTOMS748),
        "Root search method to use", enumRootsearchMethod, 0);
    enumRootsearchMethod.clear();

    env->settings->createSetting("Subsolver.Rootsearch.TerminationTolerance", 1e-16,
        "Epsilon lambda tolerance for root search", 0.0, SHOT_DBL_MAX);

    // Termination settings

    env->settings->createSettingGroup(
        "Termination", "", "Termination", "These settings control when SHOT will terminate the solution process.");

    env->settings->createSetting("Termination.ConstraintTolerance", 1e-8, "Termination tolerance for nonlinear constraints", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Termination.ObjectiveConstraintTolerance", 1e-8,
        "Termination tolerance for the nonlinear objective constraint", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Termination.IterationLimit", 200000, "Iteration limit for main strategy", 1, SHOT_INT_MAX);

    env->settings->createSetting("Termination.ObjectiveGap.Absolute", 0.001,
        "Absolute gap termination tolerance for objective function", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Termination.ObjectiveGap.Relative", 0.001,
        "Relative gap termination tolerance for objective function", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Termination.DualStagnation.ConstraintTolerance", 1e-6,
        "Min absolute difference between max nonlinear constraint errors in subsequent iterations for termination", 0,
        SHOT_DBL_MAX);

    env->settings->createSetting("Termination.DualStagnation.IterationLimit", SHOT_INT_MAX,
        "Max number of iterations without significant dual objective value improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("Termination.PrimalStagnation.IterationLimit", 50,
        "Max number of iterations without significant primal objective value improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("Termination.TimeLimit", SHOT_DBL_MAX, "Time limit (s) for solver", 0.0, SHOT_DBL_MAX);

    // Hidden settings for problem information

    VectorString enumFileFormat;
    enumFileFormat.push_back("OSiL");
    enumFileFormat.push_back("GAMS");
    enumFileFormat.push_back("NL");
    enumFileFormat.push_back("None");
    env->settings->createSetting("Input.ModelingSystem", static_cast<int>(ES_ModelingSystem::None),
        "The format of the problem file", enumFileFormat, 0, true);
    enumFileFormat.clear();

    env->settings->createSetting("Input.ProblemFile", empty, "The filename of the problem", true);

    env->settings->createSetting("Input.ProblemName", empty, "The name of the problem instance", true);

    env->settings->createSetting("Input.OptionsFile", empty, "The name of the options file used", true);

    env->settings->createSetting("Output.ResultPath", empty, "The path where to save the result information", true);

    ModelingSystemOSiL::augmentSettings(env->settings);

#ifdef HAS_AMPL
    ModelingSystemAMPL::augmentSettings(env->settings);
#endif

#ifdef HAS_GAMS
    ModelingSystemGAMS::augmentSettings(env->settings);
#endif

    env->settings->settingsInitialized = true;

    env->output->outputDebug(" Initialization of settings complete.");
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

    bool NLPSolverDefined = true;

#ifndef HAS_IPOPT
    if(static_cast<ES_PrimalNLPSolver>(env->settings->getSetting<int>("Primal.FixedInteger.Solver"))
        == ES_PrimalNLPSolver::Ipopt)
    {
        env->output->outputWarning(" SHOT has not been compiled with support for Ipopt NLP solver.");
        NLPSolverDefined = false;
    }
#endif

#ifndef HAS_GAMS
    if(static_cast<ES_PrimalNLPSolver>(env->settings->getSetting<int>("Primal.FixedInteger.Solver"))
        == ES_PrimalNLPSolver::GAMS)
    {
        env->output->outputWarning(" SHOT has not been compiled with support for GAMS NLP solvers.");
        NLPSolverDefined = false;
    }
#endif

#ifdef HAS_GAMS
    if((static_cast<ES_PrimalNLPSolver>(env->settings->getSetting<int>("Primal.FixedInteger.Solver"))
           == ES_PrimalNLPSolver::GAMS)
        && (static_cast<ES_PrimalNLPProblemSource>(
                env->settings->getSetting<int>("Primal.FixedInteger.SourceProblem"))
            != ES_PrimalNLPProblemSource::OriginalProblem))
    {
        env->output->outputWarning(" Cannot use GAMS NLP solvers when solving fixed NLP problems based on the "
                                   "reformulated model. Use Ipopt instead!");
        env->settings->updateSetting("Primal.FixedInteger.SourceProblem", static_cast<int>(ES_PrimalNLPProblemSource::OriginalProblem),
            E_SettingPriority::SolverCompatibility);
    }
#endif

    if((env->settings->getSetting<int>("Input.ModelingSystem") == static_cast<int>(ES_ModelingSystem::OSiL)
           || env->settings->getSetting<int>("Input.ModelingSystem") == static_cast<int>(ES_ModelingSystem::AMPL)
           || env->settings->getSetting<int>("Input.ModelingSystem") == static_cast<int>(ES_ModelingSystem::None))
        && static_cast<ES_PrimalNLPSolver>(env->settings->getSetting<int>("Primal.FixedInteger.Solver"))
            == ES_PrimalNLPSolver::GAMS)
    {
        env->output->outputWarning(
            " Cannot use GAMS NLP solvers with problems not originating from GAMS (OSiL, nl, or API-built problems).");
        NLPSolverDefined = false;
    }

    if(!NLPSolverDefined)
    {
#ifdef HAS_IPOPT
        env->settings->updateSetting("Primal.FixedInteger.Solver", (int)ES_PrimalNLPSolver::Ipopt,
            E_SettingPriority::SolverCompatibility);
        env->output->outputWarning(" Using Ipopt as NLP solver instead.");

#elif HAS_GAMS
        env->settings->updateSetting("Primal.FixedInteger.Solver", (int)ES_PrimalNLPSolver::GAMS,
            E_SettingPriority::SolverCompatibility);
        env->output->outputWarning(" Using GAMS NLP solvers instead.");

#else
        env->settings->updateSetting("Primal.FixedInteger.Solver", (int)ES_PrimalNLPSolver::SHOT,
            E_SettingPriority::SolverCompatibility);
        env->output->outputWarning(" No external NLP solver available. Using SHOT as NLP solver.");
#endif
    }

    // Checking for errors in MIP solver selection

    auto solver = static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver"));
    bool MIPSolverDefined = false;
    double unboundedVariableBound = 1e50;

#ifdef HAS_CPLEX
    if(solver == ES_MIPSolver::Cplex)
    {
        MIPSolverDefined = true;
        unboundedVariableBound = 1e20;

        //TODO: needed
        if(env->settings->getSetting<int>("Model.Reformulation.Quadratics.ExtractStrategy")
            > (int)ES_QuadraticTermsExtractStrategy::ExtractTermsToSame)
            env->settings->updateSetting("Model.Reformulation.Quadratics.ExtractStrategy",
                (int)ES_QuadraticTermsExtractStrategy::ExtractTermsToSame,
                E_SettingPriority::RecommendedInternal);
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
        env->settings->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree),
            E_SettingPriority::SolverCompatibility);
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy", static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear),
            E_SettingPriority::SolverCompatibility);
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy", (int)ES_QuadraticTermsExtractStrategy::DoNotExtract,
            E_SettingPriority::SolverCompatibility);
    }
#endif

#ifdef HAS_HIGHS
    if(solver == ES_MIPSolver::Highs)
    {
        MIPSolverDefined = true;
        unboundedVariableBound = 1e50;

        // Some features are not available in Highs
        env->settings->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree),
            E_SettingPriority::SolverCompatibility);
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy", static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear),
            E_SettingPriority::SolverCompatibility);
        env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy", (int)ES_QuadraticTermsExtractStrategy::DoNotExtract,
            E_SettingPriority::SolverCompatibility);
    }
#endif

    if(!MIPSolverDefined)
    {
        env->output->outputWarning(" SHOT has not been compiled with support for selected MIP solver.");

#ifdef HAS_GUROBI
        env->settings->updateSetting("Dual.MIP.Solver", (int)ES_MIPSolver::Gurobi,
            E_SettingPriority::RecommendedInternal);
        unboundedVariableBound = 1e20;
#elif HAS_CPLEX
        env->settings->updateSetting("Dual.MIP.Solver", (int)ES_MIPSolver::Cplex,
            E_SettingPriority::RecommendedInternal);
        unboundedVariableBound = 1e20;
#elif HAS_HIGHS
        env->settings->updateSetting("Dual.MIP.Solver", (int)ES_MIPSolver::Highs,
            E_SettingPriority::RecommendedInternal);
        unboundedVariableBound = 1e20;
#elif HAS_CBC
        env->settings->updateSetting("Dual.MIP.Solver", (int)ES_MIPSolver::Cbc,
            E_SettingPriority::RecommendedInternal);
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
        env->settings->updateSetting("Output.Console.Iteration.Detail", static_cast<int>(ES_IterationOutputDetail::Full),
            E_SettingPriority::RecommendedInternal);

    // Set correct iteration detail output when showing primal solver output
    if(env->settings->getSetting<bool>("Output.Console.PrimalSolver.Show"))
        env->settings->updateSetting("Output.Console.Iteration.Detail", static_cast<int>(ES_IterationOutputDetail::Full),
            E_SettingPriority::RecommendedInternal);
}

void Solver::setConvexityBasedSettingsPreReformulation()
{
    if(env->settings->getSetting<bool>("Strategy.UseRecommendedSettings"))
    {
        if(env->problem->properties.convexity != E_ProblemConvexity::Convex)
        {
            env->settings->updateSetting("Model.Reformulation.Constraint.PartitionNonlinearTerms", (int)ES_PartitionNonlinearSums::IfConvex,
                E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Model.Reformulation.Constraint.PartitionQuadraticTerms", (int)ES_PartitionNonlinearSums::IfConvex,
                E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Model.Reformulation.ObjectiveFunction.PartitionNonlinearTerms",
                (int)ES_PartitionNonlinearSums::IfConvex, E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Model.Reformulation.ObjectiveFunction.PartitionQuadraticTerms",
                (int)ES_PartitionNonlinearSums::IfConvex, E_SettingPriority::RecommendedInternal);
            // env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy", 0);

#ifdef HAS_GUROBI
            if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Gurobi)
            {
#if GRB_VERSION_MAJOR < 9
                if(env->settings->getSetting<int>("Model.Reformulation.Quadratics.ExtractStrategy")
                    > (int)ES_QuadraticTermsExtractStrategy::ExtractTermsToSame)
                    env->settings->updateSetting("Model.Reformulation.Quadratics.ExtractStrategy",
                        (int)ES_QuadraticTermsExtractStrategy::ExtractTermsToSame,
                        E_SettingPriority::RecommendedInternal);
#endif

#if GRB_VERSION_MAJOR >= 9

                if(env->settings->getSetting<bool>("Strategy.UseRecommendedSettings"))
                {
                    // Activate Gurobi nonconvex MIQCQP solver for all nonconvex quadratic terms by default
                    env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy",
                        (int)ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained,
                        E_SettingPriority::RecommendedInternal);
                }
                else if(env->settings->getSetting<int>("Model.Reformulation.Quadratics.ExtractStrategy")
                    > (int)ES_QuadraticTermsExtractStrategy::ExtractTermsToSame)
                {
                    env->settings->updateSetting("Model.Reformulation.Quadratics.Strategy",
                        (int)ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained,
                        E_SettingPriority::RecommendedInternal);
                }
#endif
            }
#endif

#ifdef HAS_CBC
            if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Cbc)
            {
                env->settings->updateSetting("Model.Reformulation.Quadratics.Decomposition.Method", (int)ES_QuadraticDecomposition::LDLDecomposition,
                    E_SettingPriority::RecommendedInternal);
            }
#endif

#ifdef HAS_HIGHS
            if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Highs)
            {
                env->settings->updateSetting("Model.Reformulation.Quadratics.Decomposition.Method", (int)ES_QuadraticDecomposition::LDLDecomposition,
                    E_SettingPriority::RecommendedInternal);
            }
#endif
        }
    }
}

void Solver::setConvexityBasedSettings()
{
    if(env->settings->getSetting<bool>("Strategy.UseRecommendedSettings"))
    {
        if(env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex)
        {
            env->settings->updateSetting("Dual.ESH.InteriorPoint.CuttingPlane.IterationLimit", 50,
                E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Dual.ESH.InteriorPoint.UsePrimalSolution", 1,
                E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting("Dual.ESH.Rootsearch.UniqueConstraints", false,
                E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting("Dual.HyperplaneCuts.ConstraintSelectionFactor", 1.0,
                E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Dual.HyperplaneCuts.UseIntegerCuts", true,
                E_SettingPriority::RecommendedInternal);

            // For full nonconvex features multitree needs to be used
            env->settings->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree),
                E_SettingPriority::SolverCompatibility);

            env->settings->updateSetting("Dual.MIP.Presolve.UpdateObtainedBounds", false,
                E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Dual.MIP.SolutionLimit.Initial", SHOT_INT_MAX,
                E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting("Dual.Relaxation.Use", false,
                E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting("Model.Reformulation.Constraint.PartitionNonlinearTerms", (int)ES_PartitionNonlinearSums::IfConvex,
                E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Model.Reformulation.Constraint.PartitionQuadraticTerms", (int)ES_PartitionNonlinearSums::IfConvex,
                E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Model.Reformulation.ObjectiveFunction.PartitionNonlinearTerms",
                (int)ES_PartitionNonlinearSums::IfConvex, E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Model.Reformulation.ObjectiveFunction.PartitionQuadraticTerms",
                (int)ES_PartitionNonlinearSums::IfConvex, E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting("Primal.FixedInteger.CallStrategy", 0,
                E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Primal.FixedInteger.CreateInfeasibilityCut", false,
                E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Primal.FixedInteger.Source", 0,
                E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting("Primal.FixedInteger.OnlyUniqueIntegerCombinations", false,
                E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting("Primal.Rootsearch.Use", true,
                E_SettingPriority::RecommendedInternal);

            env->settings->updateSetting("Model.BoundTightening.FeasibilityBased.TimeLimit", 5.0,
                E_SettingPriority::RecommendedInternal);

            // Need to save these to perform dual bound updates
            // env->settings->updateSetting("Dual.HyperplaneCuts.SaveHyperplanePoints", true);

#ifdef HAS_CPLEX

            if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Cplex)
            {
                if(env->reformulatedProblem->objectiveFunction->properties.classification
                        == E_ObjectiveFunctionClassification::Quadratic
                    || env->reformulatedProblem->properties.numberOfQuadraticConstraints > 0)
                    env->settings->updateSetting("Subsolver.Cplex.OptimalityTarget", 3,
                        E_SettingPriority::RecommendedInternal);
            }

#endif
        }

#ifdef HAS_CBC
        if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Cbc)
        {
            env->settings->updateSetting("Model.Reformulation.Constraint.PartitionNonlinearTerms", (int)ES_PartitionNonlinearSums::IfConvex,
                E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Model.Reformulation.ObjectiveFunction.PartitionNonlinearTerms",
                (int)ES_PartitionNonlinearSums::IfConvex, E_SettingPriority::RecommendedInternal);
        }
#endif

#ifdef HAS_HIGHS
        if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("Dual.MIP.Solver")) == ES_MIPSolver::Highs)
        {
            env->settings->updateSetting("Model.Reformulation.Constraint.PartitionNonlinearTerms", (int)ES_PartitionNonlinearSums::IfConvex,
                E_SettingPriority::RecommendedInternal);
            env->settings->updateSetting("Model.Reformulation.ObjectiveFunction.PartitionNonlinearTerms",
                (int)ES_PartitionNonlinearSums::IfConvex, E_SettingPriority::RecommendedInternal);
        }
#endif
    }
}

VectorString Solver::getSettingIdentifiers(E_SettingType type) { return (env->settings->getSettingIdentifiers(type)); }

double Solver::getCurrentDualBound() { return (env->results->getCurrentDualBound()); }

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
#ifdef HAS_IPOPT
    solvers.push_back(ES_PrimalNLPSolver::Ipopt);
#endif
#ifdef HAS_GAMS
    solvers.push_back(ES_PrimalNLPSolver::GAMS);
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
    default:
        return false;
    }
}

} // namespace SHOT
