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

#include "../Tasks/TaskReformulateProblem.h"

#include <map>

#ifdef HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std;
#endif

#ifdef HAS_STD_EXPERIMENTAL_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental;
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

    env->timing->createTimer("ProblemInitialization", " - problem initialization");
    env->timing->createTimer("ProblemReformulation", " - problem reformulation");
    env->timing->createTimer("BoundTightening", " - bound tightening");
    env->timing->createTimer("BoundTighteningFBBTOriginal", "   - feasibility based (original problem)");
    env->timing->createTimer("BoundTighteningFBBTReformulated", "   - feasibility based (reformulated problem)");

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

    env->timing->createTimer("ProblemInitialization", " - problem initialization");
    env->timing->createTimer("ProblemReformulation", " - problem reformulation");
    env->timing->createTimer("BoundTightening", " - bound tightening");
    env->timing->createTimer("BoundTighteningFBBT", "   - feasibility based");
    env->timing->createTimer("BoundTighteningFBBTOriginal", "   - feasibility based (original problem");
    env->timing->createTimer("BoundTighteningFBBTReformulated", "   - feasibility based (reformulated problem");

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
                "Error when reading options from \"" + fileName + "\". File extension must be osol, xml or opt.");
            result = false;
        }
    }
    catch(const std::exception& e)
    {
        env->output->outputError("Error when reading options from \"" + fileName + "\"", e.what());
        result = false;
    }

    env->settings->updateSetting("OptionsFile", "Input", fileName);

    env->output->outputDebug("Options read from file \"" + fileName + "\"");

    return (result);
}

bool Solver::setOptionsFromString(std::string options)
{
    bool status = env->settings->readSettingsFromString(options);

    env->output->outputDebug("Options read.");

    return (status);
}

bool Solver::setOptionsFromOSoL(std::string options)
{
    bool status = env->settings->readSettingsFromOSoL(options);

    verifySettings();

    env->output->outputDebug("Options read.");

    return (status);
}

bool Solver::setLogFile(std::string filename)
{
    env->output->setFileSink(filename);
    return (true);
}

bool Solver::setProblem(std::string fileName)
{
    if(!fs::filesystem::exists(fileName))
    {
        env->output->outputError("Problem file \"" + fileName + "\" does not exist.");

        return (false);
    }

    fs::filesystem::path problemFile(fileName);

    if(!problemFile.has_extension())
    {
        env->output->outputError("Problem file \"" + fileName + "\" does not specify a file extension.");

        return (false);
    }

    fs::filesystem::path problemExtension = problemFile.extension();
    fs::filesystem::path problemPath = problemFile.parent_path();

    env->settings->updateSetting("ProblemFile", "Input", fs::filesystem::absolute(problemFile).string());

    // Removes path
    fs::filesystem::path problemName = problemFile.stem();
    env->settings->updateSetting("ProblemName", "Input", problemName.string());
    env->settings->updateSetting("ProblemFile", "Input", fs::filesystem::absolute(problemFile).string());

    // Sets the debug path if not already set
    if(env->settings->getSetting<std::string>("Debug.Path", "Output") == "")
    {
        if(static_cast<ES_OutputDirectory>(env->settings->getSetting<int>("OutputDirectory", "Output"))
            == ES_OutputDirectory::Program)
        {
            fs::filesystem::path debugPath(fs::filesystem::current_path());
            debugPath /= ("debug" / problemName);

            env->settings->updateSetting("Debug.Path", "Output", debugPath.string());
        }
        else
        {
            fs::filesystem::path debugPath(problemPath);
            debugPath /= ("debug" / problemName);

            env->settings->updateSetting("Debug.Path", "Output", debugPath.string());
        }
    }

    // Sets the result path
    if(static_cast<ES_OutputDirectory>(env->settings->getSetting<int>("OutputDirectory", "Output"))
        == ES_OutputDirectory::Program)
    {
        env->settings->updateSetting("ResultPath", "Output", fs::filesystem::current_path().string());
    }
    else
    {
        env->settings->updateSetting("ResultPath", "Output", problemPath.string());
    }

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        initializeDebugMode();
    }

    // Do not do convexifying reformulations if the problem is assumed to be convex
    if(env->settings->getSetting<bool>("Convexity.AssumeConvex", "Model"))
    {
        env->settings->updateSetting(
            "Reformulation.Bilinear.IntegerFormulation", "Model", (int)ES_ReformulatiomBilinearInteger::None);

        env->settings->updateSetting(
            "Reformulation.Monomials.Formulation", "Model", (int)ES_ReformulatiomBilinearInteger::None);
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
    if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("MIP.Solver", "Dual")) == ES_MIPSolver::Cbc)
    {
        env->settings->updateSetting(
            "Reformulation.Quadratics.Strategy", "Model", (int)ES_QuadraticProblemStrategy::Nonlinear);
    }
#endif

    try
    {
        if(problemExtension == ".osil" || problemExtension == ".xml")
        {
            env->report->outputModelingSystemHeader(ES_SourceFormat::OSiL, fileName);

            auto modelingSystem = std::make_shared<ModelingSystemOSiL>(env);
            ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

            if(modelingSystem->createProblem(problem, fileName) != E_ProblemCreationStatus::NormalCompletion)
            {
                return (false);
            }

            env->modelingSystem = modelingSystem;
            env->problem = problem;

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::OSiL));
        }

#ifdef HAS_AMPL
        if(problemExtension == ".nl")
        {
            env->report->outputModelingSystemHeader(ES_SourceFormat::NL, fileName);

            auto modelingSystem = std::make_shared<ModelingSystemAMPL>(env);
            ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

            if(modelingSystem->createProblem(problem, fileName) != E_ProblemCreationStatus::NormalCompletion)

            {
                return (false);
            }

            env->modelingSystem = modelingSystem;
            env->problem = problem;

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::NL));
        }
#endif

#ifdef HAS_GAMS
        if(problemExtension == ".gms")
        {
            env->report->outputModelingSystemHeader(ES_SourceFormat::GAMS, fileName);

            auto modelingSystem = std::make_shared<SHOT::ModelingSystemGAMS>(env);
            SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

            if(modelingSystem->createProblem(problem, fileName, E_GAMSInputSource::ProblemFile)
                != E_ProblemCreationStatus::NormalCompletion)
            {
                return (false);
            }

            env->modelingSystem = modelingSystem;
            env->problem = problem;

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::GAMS));
        }
#endif

        if(env->problem->name == "")
            env->problem->name = problemName.string();

        auto taskReformulateProblem = std::make_unique<TaskReformulateProblem>(env);
        taskReformulateProblem->run();

        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            std::stringstream problemFilename;
            problemFilename << env->settings->getSetting<std::string>("Debug.Path", "Output");
            problemFilename << "/originalproblem.txt";

            std::stringstream problemText;
            problemText << env->problem;

            Utilities::writeStringToFile(problemFilename.str(), problemText.str());
        }
    }
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format("Error when reading problem from \"{0}\"", e.what()));

        return (false);
    }

    verifySettings();
    setConvexityBasedSettings();

    return (this->selectStrategy());
}

bool Solver::setProblem(SHOT::ProblemPtr problem, SHOT::ModelingSystemPtr modelingSystem)
{
    env->modelingSystem = modelingSystem;
    env->problem = problem;

    env->settings->updateSetting("ProblemName", "Input", problem->name);

    if(static_cast<ES_OutputDirectory>(env->settings->getSetting<int>("OutputDirectory", "Output"))
        == ES_OutputDirectory::Program)
    {
        fs::filesystem::path debugPath(fs::filesystem::current_path());
        debugPath /= problem->name;

        env->settings->updateSetting("Debug.Path", "Output", debugPath.string());
        env->settings->updateSetting("ResultPath", "Output", fs::filesystem::current_path().string());
    }

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        initializeDebugMode();

        std::stringstream filename;
        filename << env->settings->getSetting<std::string>("Debug.Path", "Output");
        filename << "/originalproblem";
        filename << ".txt";

        std::stringstream problem;
        problem << env->problem;

        Utilities::writeStringToFile(filename.str(), problem.str());
    }

    // Do not do convexifying reformulations if the problem is assumed to be convex
    if(env->settings->getSetting<bool>("Convexity.AssumeConvex", "Model"))
    {
        env->settings->updateSetting(
            "Reformulation.Bilinear.IntegerFormulation", "Model", (int)ES_ReformulatiomBilinearInteger::None);

        env->settings->updateSetting(
            "Reformulation.Monomials.Formulation", "Model", (int)ES_ReformulatiomBilinearInteger::None);
    }

#ifdef HAS_CBC
    // TODO: figure out a better way to do this
    if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("MIP.Solver", "Dual")) == ES_MIPSolver::Cbc)
    {
        env->settings->updateSetting(
            "Reformulation.Quadratics.Strategy", "Model", (int)ES_QuadraticProblemStrategy::Nonlinear);
    }
#endif

    auto taskReformulateProblem = std::make_unique<TaskReformulateProblem>(env);
    taskReformulateProblem->run();

    verifySettings();
    setConvexityBasedSettings();

    return (this->selectStrategy());
}

bool Solver::selectStrategy()
{
    try
    {
        if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("MIP.Solver", "Dual")) == ES_MIPSolver::Cbc)
        {
            if(env->problem->properties.numberOfDiscreteVariables == 0)
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
            env->settings->getSetting<int>("Reformulation.Quadratics.Strategy", "Model"));
        bool useQuadraticConstraints
            = (quadraticStrategy >= ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained);
        bool useQuadraticObjective
            = (useQuadraticConstraints || quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticObjective);

        bool isConvex = env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Convex;

        if(isConvex && (useQuadraticObjective || useQuadraticConstraints) && env->problem->properties.isMIQPProblem)
        // Convex MIQP problem
        {
            env->output->outputDebug(" Using convex MIQP solution strategy.");
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQP;
        }
        else if(isConvex && (useQuadraticObjective || useQuadraticConstraints) && env->problem->properties.isQPProblem)
        // Convex QP problem
        {
            env->output->outputDebug(" Using convex QP solution strategy.");
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQP;
        }
        // Convex MIQCQP problem
        else if(isConvex && useQuadraticConstraints && env->problem->properties.isMIQCQPProblem)
        {
            env->output->outputDebug(" Using convex MIQCQP solution strategy.");
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQCQP;
        }
        // Convex QCQP problem
        else if(isConvex && (useQuadraticConstraints || useQuadraticConstraints)
            && env->problem->properties.isQCQPProblem)
        {
            env->output->outputDebug(" Using convex QCQP solution strategy.");
            solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::MIQCQP;
        }
        // MILP problem
        else if(env->problem->properties.isMILPProblem || env->problem->properties.isLPProblem)
        {
            env->output->outputDebug(" Using MILP solution strategy.");
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
                switch(static_cast<ES_TreeStrategy>(env->settings->getSetting<int>("TreeStrategy", "Dual")))
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
    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        std::stringstream filename;
        filename << env->settings->getSetting<std::string>("Debug.Path", "Output");
        filename << "/usedsettings";
        filename << ".opt";

        auto usedSettings = env->settings->getSettingsAsString(false, false);

        Utilities::writeStringToFile(filename.str(), usedSettings);
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

    return (isProblemSolved);
}

void Solver::finalizeSolution()
{
    if(env->modelingSystem)
        env->modelingSystem->finalizeSolution();
}

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
        env->output->outputWarning("Warning! Settings have already been initialized. Ignoring new settings.");
        return;
    }

    std::string empty; // Used to create empty string options

    env->output->outputDebug("Starting initialization of settings:");

    env->settings->createSettingGroup("Dual", "", "Dual strategy",
        "These settings control the various functionality of the dual strategy in SHOT, i.e., the polyhedral outer "
        "approximation utilizing the ESH or ECP algorithms.");

    // Dual strategy settings: ECP and ESH

    VectorString enumHyperplanePointStrategy;
    enumHyperplanePointStrategy.push_back("ESH");
    enumHyperplanePointStrategy.push_back("ECP");
    env->settings->createSetting("CutStrategy", "Dual", static_cast<int>(ES_HyperplaneCutStrategy::ESH),
        "Dual cut strategy", enumHyperplanePointStrategy);
    enumHyperplanePointStrategy.clear();

    env->settings->createSettingGroup("Dual", "ESH", "Extended supporting hyperplane method",
        "These settings control various aspects of the ESH implementation, including the strategy to obtain the "
        "interior point.");

    // Dual strategy settings: Interior point search strategy

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.BitPrecision", "Dual", 8,
        "Required termination bit precision for minimization subsolver", 1, 64, true);

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.ConstraintSelectionFactor", "Dual", 0.25,
        "The fraction of violated constraints to generate cutting planes for", 0.0, 1.0);

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.IterationLimit", "Dual", 100,
        "Iteration limit for minimax cutting plane solver", 1, SHOT_INT_MAX);

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.IterationLimitSubsolver", "Dual", 100,
        "Iteration limit for minimization subsolver", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "ESH.InteriorPoint.CuttingPlane.TimeLimit", "Dual", 10.0, "Time limit for minimax solver", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "ESH.InteriorPoint.CuttingPlane.Reuse", "Dual", false, "Reuse valid cutting planes in main dual model");

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceAbs", "Dual", 1.0,
        "Absolute termination tolerance between LP and linesearch objective", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceRel", "Dual", 1.0,
        "Relative termination tolerance between LP and linesearch objective", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("ESH.InteriorPoint.MinimaxObjectiveLowerBound", "Dual", -1e12,
        "Lower bound for minimax objective variable", SHOT_DBL_MIN, 0);

    env->settings->createSetting("ESH.InteriorPoint.MinimaxObjectiveUpperBound", "Dual", 0.1,
        "Upper bound for minimax objective variable", SHOT_DBL_MIN, SHOT_DBL_MAX);

    VectorString enumNLPSolver;
    enumNLPSolver.push_back("Cutting plane minimax");
    /*enumNLPSolver.push_back("Ipopt minimax");
    enumNLPSolver.push_back("Ipopt relaxed");
    enumNLPSolver.push_back("Ipopt minimax and relaxed");*/

    env->settings->createSetting("ESH.InteriorPoint.Solver", "Dual",
        static_cast<int>(ES_InteriorPointStrategy::CuttingPlaneMiniMax), "NLP solver", enumNLPSolver, true);
    enumNLPSolver.clear();

    VectorString enumAddPrimalPointAsInteriorPoint;
    enumAddPrimalPointAsInteriorPoint.push_back("No");
    enumAddPrimalPointAsInteriorPoint.push_back("Add as new");
    enumAddPrimalPointAsInteriorPoint.push_back("Replace old");
    enumAddPrimalPointAsInteriorPoint.push_back("Use avarage");
    env->settings->createSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual",
        static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepBoth), "Utilize primal solution as interior point",
        enumAddPrimalPointAsInteriorPoint);
    enumAddPrimalPointAsInteriorPoint.clear();

    env->settings->createSetting(
        "ESH.Rootsearch.UniqueConstraints", "Dual", false, "Allow only one hyperplane per constraint per iteration");

    env->settings->createSetting("ESH.Rootsearch.ConstraintTolerance", "Dual", 1e-8,
        "Constraint tolerance for when not to add individual hyperplanes", 0, SHOT_DBL_MAX);

    // Dual strategy settings: Fixed integer strategy

    env->settings->createSettingGroup("Dual", "FixedInteger", "Fixed integer dual strategy",
        "If no progress has been made in a certain number of iterations, it is possible to fix the integer variables "
        "and solve integer-relaxed subproblems. These settings control this functionality. ");

    env->settings->createSetting("FixedInteger.ConstraintTolerance", "Dual", 0.0001,
        "Constraint tolerance for fixed strategy", 0.0, SHOT_DBL_MAX, true);

    env->settings->createSetting(
        "FixedInteger.MaxIterations", "Dual", 20, "Max LP iterations for fixed strategy", 0, SHOT_INT_MAX, true);

    env->settings->createSetting("FixedInteger.ObjectiveTolerance", "Dual", 0.001,
        "Objective tolerance for fixed strategy", 0.0, SHOT_DBL_MAX, true);

    env->settings->createSetting("FixedInteger.Use", "Dual", false,
        "Solve a fixed LP problem if integer-values have not changes in several MIP iterations", true);

    // Dual strategy settings: Hyperplane generation

    env->settings->createSettingGroup("Dual", "HyperplaneCuts", "Generated hyperplane cuts",
        "These settings control how the cutting planes or supporting hyperplanes are generated.");

    env->settings->createSetting("HyperplaneCuts.ConstraintSelectionFactor", "Dual", 0.5,
        "The fraction of violated constraints to generate supporting hyperplanes / cutting planes for", 0.0, 1.0);

    env->settings->createSetting(
        "HyperplaneCuts.Delay", "Dual", true, "Add hyperplane cuts to model only after optimal MIP solution");

    env->settings->createSetting("HyperplaneCuts.MaxConstraintFactor", "Dual", 0.1,
        "Rootsearch performed on constraints with values larger than this factor times the maximum value", 1e-6, 1.0);

    env->settings->createSetting("HyperplaneCuts.MaxPerIteration", "Dual", 200,
        "Maximal number of hyperplanes to add per iteration", 0, SHOT_INT_MAX);

    env->settings->createSetting("HyperplaneCuts.UseIntegerCuts", "Dual", false,
        "Add integer cuts for infeasible integer-combinations for binary problems");

    VectorString enumObjectiveRootsearch;
    enumObjectiveRootsearch.push_back("Always");
    enumObjectiveRootsearch.push_back("IfConvex");
    enumObjectiveRootsearch.push_back("Never");
    env->settings->createSetting("HyperplaneCuts.ObjectiveRootSearch", "Dual",
        static_cast<int>(ES_ObjectiveRootsearch::IfConvex), "When to use the objective root search",
        enumObjectiveRootsearch);
    enumObjectiveRootsearch.clear();

    // TODO: activate
    // env->settings->createSetting(
    //    "HyperplaneCuts.UsePrimalObjectiveCut", "Dual", true, "Add an objective cut in the primal solution");

    // Dual strategy settings: MIP solver

    env->settings->createSettingGroup("Dual", "MIP", "MIP solver",
        "These settings control the general functionality of the MIP solver in the dual strategy. Note that "
        "solver-specific settings for Cplex, Gurobi and Cbc are available under the \"Subsolver\" category.");

    env->settings->createSetting(
        "MIP.CutOff.InitialValue", "Dual", SHOT_DBL_MAX, "Initial cutoff value to use", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSetting("MIP.CutOff.UseInitialValue", "Dual", false, "Use the initial cutoff value");

    env->settings->createSetting("MIP.CutOff.Tolerance", "Dual", 0.00001,
        "An extra tolerance for the objective cutoff value (to prevent infeasible subproblems)", SHOT_DBL_MIN,
        SHOT_DBL_MAX);

    env->settings->createSetting(
        "MIP.InfeasibilityRepair.IntegerCuts", "Dual", true, "Allow feasibility repair of integer cuts");

    env->settings->createSetting("MIP.InfeasibilityRepair.IterationLimit", "Dual", 100,
        "Max number of infeasible problems repaired without primal objective value improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("MIP.InfeasibilityRepair.TimeLimit", "Dual", 10.0,
        "Time limit when reparing infeasible problem", 0, SHOT_DBL_MAX);

    env->settings->createSetting("MIP.NodeLimit", "Dual", SHOT_DBL_MAX,
        "Node limit to use for MIP solver in single-tree strategy", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "MIP.NumberOfThreads", "Dual", 0, "Number of threads to use in MIP solver: 0: Automatic", 0, 999);

    VectorString enumPresolve;
    enumPresolve.push_back("Never");
    enumPresolve.push_back("Once");
    enumPresolve.push_back("Always");
    env->settings->createSetting("MIP.Presolve.Frequency", "Dual", static_cast<int>(ES_MIPPresolveStrategy::Once),
        "When to call the MIP presolve", enumPresolve);
    enumPresolve.clear();

    env->settings->createSetting("MIP.Presolve.RemoveRedundantConstraints", "Dual", false,
        "Remove redundant constraints (as determined by presolve)");

    env->settings->createSetting(
        "MIP.Presolve.UpdateObtainedBounds", "Dual", true, "Update bounds (from presolve) to the MIP model");

    env->settings->createSetting("MIP.SolutionLimit.ForceOptimal.Iteration", "Dual", 10000,
        "Iterations without dual bound updates for forcing optimal MIP solution", 0, SHOT_INT_MAX);

    env->settings->createSetting("MIP.SolutionLimit.ForceOptimal.Time", "Dual", 1000.0,
        "Time (s) without dual bound updates for forcing optimal MIP solution", 0, SHOT_DBL_MAX);

    env->settings->createSetting("MIP.SolutionLimit.IncreaseIterations", "Dual", 50,
        "Max number of iterations between MIP solution limit increases", 0, SHOT_INT_MAX);

    env->settings->createSetting("MIP.SolutionLimit.Initial", "Dual", 1, "Initial MIP solution limit", 1, SHOT_INT_MAX);

    env->settings->createSetting("MIP.SolutionLimit.UpdateTolerance", "Dual", 0.001,
        "The constraint tolerance for when to update MIP solution limit", 0, SHOT_DBL_MAX);

    env->settings->createSetting("MIP.SolutionPool.Capacity", "Dual", 100,
        "The maximum number of solutions in the solution pool", 0, SHOT_INT_MAX);

    VectorString enumMIPSolver;
    enumMIPSolver.push_back("Cplex");
    enumMIPSolver.push_back("Gurobi");
    enumMIPSolver.push_back("Cbc");
    env->settings->createSetting(
        "MIP.Solver", "Dual", static_cast<int>(ES_MIPSolver::Cplex), "Which MIP solver to use", enumMIPSolver);
    enumMIPSolver.clear();

    env->settings->createSetting(
        "MIP.UpdateObjectiveBounds", "Dual", false, "Update nonlinear objective variable bounds to primal/dual bounds");

    // Primal settings: reduction cuts for nonconvex problems

    env->settings->createSettingGroup("Dual", "ReductionCut", "Dual reduction cut",
        "These settings control the added dual reduction cuts from the primal solution that will try to force a better "
        "primal solution. This functionality is only used if SHOT cannot deduce that the problem is nonconvex .");

    env->settings->createSetting("ReductionCut.MaxIterations", "Dual", 5,
        "Max number of primal cut reduction without primal improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "ReductionCut.ReductionFactor", "Dual", 0.001, "The factor used to reduce the cutoff value", 0, 1.0);

    // Dual strategy settings: Relaxation strategies

    env->settings->createSettingGroup("Dual", "Relaxation", "Relaxation strategies",
        "These settings contorl various aspects regarding integer-relaxation of the dual problem.");

    env->settings->createSetting("Relaxation.Use", "Dual", true, "Initially solve continuous dual relaxations");

    env->settings->createSetting(
        "Relaxation.Frequency", "Dual", 0, "The frequency to solve an LP problem: 0: Disable", 0, SHOT_INT_MAX);

    env->settings->createSetting("Relaxation.IterationLimit", "Dual", 200,
        "The max number of relaxed LP problems to solve initially", 0, SHOT_INT_MAX);

    env->settings->createSetting("Relaxation.MaxLazyConstraints", "Dual", 0,
        "Max number of lazy constraints to add in relaxed solutions in single-tree strategy", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "Relaxation.TerminationTolerance", "Dual", 0.5, "Time limit (s) when solving LP problems initially");

    env->settings->createSetting(
        "Relaxation.TimeLimit", "Dual", 30.0, "Time limit (s) when solving LP problems initially", 0, SHOT_DBL_MAX);

    // Dual strategy settings: Main tree strategy

    env->settings->createSettingGroup("Dual", "TreeStrategy", "Tree strategy",
        "The single-tree strategy is normally more efficient than the multi-tree one. However, not all MIP solvers "
        "support the required lazy constraint callbacks. These settings selects this strategy and controls its "
        "behaviour.");

    VectorString enumSolutionStrategy;
    enumSolutionStrategy.push_back("Multi-tree");
    enumSolutionStrategy.push_back("Single-tree");
    env->settings->createSetting("TreeStrategy", "Dual", static_cast<int>(ES_TreeStrategy::SingleTree),
        "The main strategy to use", enumSolutionStrategy);
    enumSolutionStrategy.clear();

    env->settings->createSetting("TreeStrategy.Multi.Reinitialize", "Dual", false,
        "Reinitialize the dual model in the subsolver each iteration", true);

    // Optimization model settings

    env->settings->createSettingGroup("Model", "", "Optimization model",
        "These settings control various aspects of SHOT's representation  for and handling of the provided "
        "optimization model.");

    // Bound tightening

    env->settings->createSettingGroup("Model", "BoundTightening", "Bound tightening",
        "SHOT performs bound tightening to strengthen the internal representation of the problem. These settings "
        "control how and when bound tightening is performed.");

    env->settings->createSetting(
        "BoundTightening.FeasibilityBased.MaxIterations", "Model", 5, "Maximal number of bound tightening iterations");

    env->settings->createSetting("BoundTightening.FeasibilityBased.TimeLimit", "Model", 5.0,
        "Time limit for bound tightening", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "BoundTightening.FeasibilityBased.Use", "Model", true, "Peform feasibility-based bound tightening");

    env->settings->createSetting("BoundTightening.FeasibilityBased.UseNonlinear", "Model", true,
        "Peform feasibility-based bound tightening on nonlinear expressions");

    env->settings->createSettingGroup(
        "Model", "Convexity", "Convexity", "These settings control the convexity detection functionality.");

    env->settings->createSetting("Convexity.AssumeConvex", "Model", false, "Assume that the problem is convex.");

    env->settings->createSetting("Convexity.Quadratics.EigenValueTolerance", "Model", 1e-5,
        "Convexity tolerance for the eigenvalues of the Hessian matrix for quadratic terms", 0.0, SHOT_DBL_MAX);

    env->settings->createSettingGroup("Model", "Variables", "Variables",
        "These settings control the maximum variable bounds allowed in SHOT. Projection will be performed onto these "
        "intervals. Note that the MIP solvers may have stricter requirements, in which case those may be used.");

    env->settings->createSetting("Variables.Continuous.MinimumLowerBound", "Model", -1e50,
        "Minimum lower bound for continuous variables", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSetting("Variables.Continuous.MaximumUpperBound", "Model", 1e50,
        "Maximum upper bound for continuous variables", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSetting("Variables.Integer.MinimumLowerBound", "Model", -2.0e9,
        "Minimum lower bound for integer variables", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSetting("Variables.Integer.MaximumUpperBound", "Model", 2.0e9,
        "Maximum upper bound for integer variables", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSetting("Variables.NonlinearObjectiveVariable.Bound", "Model", 1e12,
        "Max absolute bound for the auxiliary nonlinear objective variable", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSettingGroup("Model", "Reformulation", "Automatic reformulations",
        "These settings control the automatic reformulations performed in SHOT.");

    // Reformulations for bilinears
    env->settings->createSetting("Reformulation.Bilinear.AddConvexEnvelope", "Model", false,
        "Add convex envelopes (subject to original bounds) to bilinear terms");

    // Reformulations for integer bilinears
    VectorString enumBilinearIntegerReformulation;
    enumBilinearIntegerReformulation.push_back("None");
    enumBilinearIntegerReformulation.push_back("1D");
    enumBilinearIntegerReformulation.push_back("2D");
    env->settings->createSetting("Reformulation.Bilinear.IntegerFormulation", "Model",
        static_cast<int>(ES_ReformulatiomBilinearInteger::OneDiscretization),
        "How to reformulate integer bilinear terms", enumBilinearIntegerReformulation);
    enumBilinearIntegerReformulation.clear();

    env->settings->createSetting("Reformulation.Bilinear.IntegerFormulation.MaxVariableDomain", "Model", 100,
        "Do not reformulate integer variables in bilinear terms which can assume more than this number of discrete "
        "values",
        2, SHOT_INT_MAX);

    // Reformulations for constraints
    VectorString enumNonlinearTermPartitioning;
    enumNonlinearTermPartitioning.push_back("Always");
    enumNonlinearTermPartitioning.push_back("If convex");
    enumNonlinearTermPartitioning.push_back("Never");
    env->settings->createSetting("Reformulation.Constraint.PartitionNonlinearTerms", "Model",
        static_cast<int>(ES_PartitionNonlinearSums::IfConvex), "When to partition nonlinear sums in objective function",
        enumNonlinearTermPartitioning);

    env->settings->createSetting("Reformulation.Constraint.PartitionQuadraticTerms", "Model",
        static_cast<int>(ES_PartitionNonlinearSums::IfConvex), "When to partition quadratic sums in objective function",
        enumNonlinearTermPartitioning);

    // Reformulations for monomials

    env->settings->createSetting(
        "Reformulation.Monomials.Extract", "Model", true, "Extract monomial terms from nonlinear expressions");

    VectorString enumBinaryMonomialReformulation;
    enumBinaryMonomialReformulation.push_back("None");
    enumBinaryMonomialReformulation.push_back("Simple");
    enumBinaryMonomialReformulation.push_back("Costa and Liberti");
    env->settings->createSetting("Reformulation.Monomials.Formulation", "Model",
        static_cast<int>(ES_ReformulationBinaryMonomials::Simple), "How to reformulate binary monomials",
        enumBinaryMonomialReformulation);
    enumBinaryMonomialReformulation.clear();

    // Reformulations for objective functions
    env->settings->createSetting("Reformulation.ObjectiveFunction.Epigraph.Use", "Model", false,
        "Reformulates a nonlinear objective as an auxiliary constraint");

    env->settings->createSetting("Reformulation.ObjectiveFunction.PartitionNonlinearTerms", "Model",
        static_cast<int>(ES_PartitionNonlinearSums::IfConvex), "When to partition nonlinear sums in objective function",
        enumNonlinearTermPartitioning);

    env->settings->createSetting("Reformulation.ObjectiveFunction.PartitionQuadraticTerms", "Model",
        static_cast<int>(ES_PartitionNonlinearSums::IfConvex), "When to partition quadratic sums in objective function",
        enumNonlinearTermPartitioning);

    enumNonlinearTermPartitioning.clear();

    // Reformulations for signomials

    env->settings->createSetting(
        "Reformulation.Signomials.Extract", "Model", true, "Extract signomial terms from nonlinear expressions");

    // Reformulations for quadratic objective and constraints

    VectorString enumQuadExtractStrategy;
    enumQuadExtractStrategy.push_back("Do not extract");
    enumQuadExtractStrategy.push_back("Extract to same objective or constraint");
    enumQuadExtractStrategy.push_back("Extract to quadratic equality constraint if nonconvex");
    enumQuadExtractStrategy.push_back("Extract to quadratic equality constraint even if convex");

    env->settings->createSetting("Reformulation.Quadratics.ExtractStrategy", "Model",
        static_cast<int>(ES_QuadraticTermsExtractStrategy::ExtractTermsToSame),
        "How to extract quadratic terms from nonlinear expressions", enumQuadExtractStrategy);
    enumQuadExtractStrategy.clear();

    VectorString enumQPStrategy;
    enumQPStrategy.push_back("All nonlinear");
    enumQPStrategy.push_back("Use quadratic objective");
    enumQPStrategy.push_back("Use convex quadratic objective and constraints");
    enumQPStrategy.push_back("Use nonconvex quadratic objective and constraints");

    env->settings->createSetting("Reformulation.Quadratics.Strategy", "Model",
        static_cast<int>(ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained),
        "How to treat quadratic functions", enumQPStrategy);
    enumQPStrategy.clear();

    // Logging and output settings

    env->settings->createSettingGroup("Output", "", "Solver output",
        "These settings control how much and what output is shown to the user from the solver.");

    VectorString enumLogLevel;
    enumLogLevel.push_back("Trace");
    enumLogLevel.push_back("Debug");
    enumLogLevel.push_back("Info");
    enumLogLevel.push_back("Warning");
    enumLogLevel.push_back("Error");
    enumLogLevel.push_back("Critical");
    enumLogLevel.push_back("Off");
    env->settings->createSetting(
        "Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Info), "Log level for console output", enumLogLevel);

    env->settings->createSetting("Debug.Enable", "Output", false, "Use debug functionality");

    env->settings->createSetting("Debug.Path", "Output", empty, "The path where to save the debug information", true);

    env->settings->createSetting(
        "File.LogLevel", "Output", static_cast<int>(E_LogLevel::Info), "Log level for file output", enumLogLevel);
    enumLogLevel.clear();

    env->settings->createSetting("Console.DualSolver.Show", "Output", false, "Show output from dual solver on console");

#ifdef HAS_GAMS

    env->settings->createSetting("Console.GAMS.Show", "Output", false, "Show GAMS output on console");

#endif

    VectorString enumIterationDetail;
    enumIterationDetail.push_back("Full");
    enumIterationDetail.push_back("On objective gap update");
    enumIterationDetail.push_back("On objective gap update and all primal NLP calls");

    env->settings->createSetting("Console.Iteration.Detail", "Output",
        static_cast<int>(ES_IterationOutputDetail::ObjectiveGapUpdates), "When should the fixed strategy be used",
        enumIterationDetail);
    enumIterationDetail.clear();

    VectorString enumOutputDirectory;
    enumOutputDirectory.push_back("Problem directory");
    enumOutputDirectory.push_back("Program directory");
    env->settings->createSetting("OutputDirectory", "Output", static_cast<int>(ES_OutputDirectory::Program),
        "Where to save the output files", enumOutputDirectory);
    enumOutputDirectory.clear();

    env->settings->createSetting(
        "SaveNumberOfSolutions", "Output", 1, "Save this number of primal solutions to OSrL file");

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

    env->settings->createSetting("FixedInteger.CallStrategy", "Primal",
        static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions),
        "When should the fixed strategy be used", enumPrimalNLPStrategy);
    enumPrimalNLPStrategy.clear();

    env->settings->createSetting(
        "FixedInteger.CreateInfeasibilityCut", "Primal", false, "Create a cut from an infeasible solution point");

    env->settings->createSetting(
        "FixedInteger.Frequency.Dynamic", "Primal", true, "Dynamically update the call frequency based on success");

    env->settings->createSetting(
        "FixedInteger.Frequency.Iteration", "Primal", 10, "Max number of iterations between calls", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "FixedInteger.Frequency.Time", "Primal", 5.0, "Max duration (s) between calls", 0, SHOT_DBL_MAX);

    env->settings->createSetting("FixedInteger.DualPointGap.Relative", "Primal", 0.001,
        "If the objective gap between the MIP point and dual solution is less than this the fixed strategy is "
        "activated",
        0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "FixedInteger.IterationLimit", "Primal", 10000000, "Max number of iterations per call", 0, SHOT_INT_MAX);

    env->settings->createSetting("FixedInteger.OnlyUniqueIntegerCombinations", "Primal", true,
        "Whether to resolve with the same integer combination, e.g. for nonconvex problems with different continuous "
        "variable starting points");

    VectorString enumPrimalNLPSolver;
    enumPrimalNLPSolver.push_back("Ipopt");
    enumPrimalNLPSolver.push_back("GAMS");

    env->settings->createSetting("FixedInteger.Solver", "Primal", static_cast<int>(ES_PrimalNLPSolver::Ipopt),
        "NLP solver to use", enumPrimalNLPSolver);
    enumPrimalNLPSolver.clear();

    VectorString enumPrimalBoundNLPStartingPoint;
    enumPrimalBoundNLPStartingPoint.push_back("All");
    enumPrimalBoundNLPStartingPoint.push_back("First");
    enumPrimalBoundNLPStartingPoint.push_back("All feasible");
    enumPrimalBoundNLPStartingPoint.push_back("First and all feasible");
    enumPrimalBoundNLPStartingPoint.push_back("With smallest constraint deviation");
    env->settings->createSetting("FixedInteger.Source", "Primal",
        static_cast<int>(ES_PrimalNLPFixedPoint::FirstAndFeasibleSolutions), "Source of fixed MIP solution point",
        enumPrimalBoundNLPStartingPoint);
    enumPrimalBoundNLPStartingPoint.clear();

    VectorString enumPrimalBoundNLPProblemSource;
    enumPrimalBoundNLPProblemSource.push_back("Original problem");
    enumPrimalBoundNLPProblemSource.push_back("Reformulated problem");
    enumPrimalBoundNLPProblemSource.push_back("Both");
    env->settings->createSetting("FixedInteger.SourceProblem", "Primal",
        static_cast<int>(ES_PrimalNLPProblemSource::OriginalProblem),
        "Which problem formulation to use for NLP problem", enumPrimalBoundNLPProblemSource);
    enumPrimalBoundNLPProblemSource.clear();

    env->settings->createSetting(
        "FixedInteger.TimeLimit", "Primal", 10.0, "Time limit (s) per NLP problem", 0, SHOT_DBL_MAX);

    env->settings->createSetting("FixedInteger.Use", "Primal", true, "Use the fixed integer primal strategy");

    env->settings->createSetting("FixedInteger.UsePresolveBounds", "Primal", false,
        "Use variable bounds from MIP in NLP problems. Warning! Does not seem to work", true);

    env->settings->createSetting("FixedInteger.ProblemSource", "Primal", true, "Use the fixed integer primal strategy");

    env->settings->createSetting("FixedInteger.Warmstart", "Primal", true, "Warm start the NLP solver");

    // Primal settings: rootsearch

    env->settings->createSettingGroup("Primal", "Rootsearch", "Primal root search",
        "SHOT can utilize root searches between the dual solution point and an integer-fixed interior point. This "
        "setting controls whether this strategy is used.");

    env->settings->createSetting("Rootsearch.Use", "Primal", true, "Use a rootsearch to find primal solutions");

    // Primal settings: tolerances for accepting primal solutions

    env->settings->createSettingGroup("Primal", "Tolerances", "Primal solution tolerances",
        "These settings sets various tolerances for accepting primal solutions.");

    env->settings->createSetting("Tolerance.TrustLinearConstraintValues", "Primal", true,
        "Trust that subsolvers (NLP, MIP) give primal solutions that respect linear constraints");

    env->settings->createSetting(
        "Tolerance.Integer", "Primal", 1e-5, "Integer tolerance for accepting primal solutions");

    env->settings->createSetting(
        "Tolerance.LinearConstraint", "Primal", 1e-6, "Linear constraint tolerance for accepting primal solutions");

    env->settings->createSetting("Tolerance.NonlinearConstraint", "Primal", 1e-5,
        "Nonlinear constraint tolerance for accepting primal solutions");

    // Strategy settings

    env->settings->createSettingGroup("Strategy", "", "Strategy", "Overall strategy parameters used in SHOT.");

    env->settings->createSetting("UseRecommendedSettings", "Strategy", true,
        "Modifies some settings to their recommended values based on the strategy");

    // Subsolver settings: Cplex

    env->settings->createSettingGroup("Subsolver", "", "Subsolver functionality",
        "These settings allow for more direct control of the  different subsolvers utilized in SHOT.");

#ifdef HAS_CPLEX

    env->settings->createSettingGroup("Subsolver", "Cplex", "Cplex", "");

    env->settings->createSetting("Cplex.AddRelaxedLazyConstraintsAsLocal", "Subsolver", false,
        "Whether to add lazy constraints generated in relaxed points as local or global");

    env->settings->createSetting("Cplex.OptimalityTarget", "Subsolver", 0,
        "Specifies how CPLEX treats nonconvex quadratics: 0. Automatic. 1. Searches for a globally optimal solution to "
        "a convex model. 2. Searches for a solution that satisfies first-order optimality "
        "conditions, but is not necessarily globally optimal. 3. Searches for a globally "
        "optimal solution to a nonconvex model; changes problem type to MIQP if necessary.",
        0, 3);

    env->settings->createSetting("Cplex.FeasOptMode", "Subsolver", 0,
        "Strategy to use for the feasibility repair: 0. Minimize the sum of all required relaxations in first phase "
        "only. 1. Minimize the sum of all required relaxations in first phase and execute second phase to find optimum "
        "among minimal relaxations. 2. Minimize the number of constraints and bounds requiring relaxation in first "
        "phase only. 3. Minimize the number of constraints and bounds requiring relaxation in first phase and execute "
        "second phase to find optimum among minimal relaxations. 4. Minimize the sum of squares of required "
        "relaxations in first phase only. 5. Minimize the sum of squares of required relaxations in first phase and "
        "execute second phase to find optimum among minimal relaxations",
        0, 5);

    env->settings->createSetting("Cplex.MemoryEmphasis", "Subsolver", 0, "Try to conserve memory when possible", 0, 1);

    env->settings->createSetting("Cplex.MIPEmphasis", "Subsolver", 0,
        "Sets the MIP emphasis: 0: Balanced. 1: Feasibility. 2: Optimality. 3: Best bound. 4: Hidden feasible", 0, 4);

    env->settings->createSetting("Cplex.NodeFile", "Subsolver", 1,
        "Where to store the node file: 0: No file. 1: Compressed in memory. 2: On disk. 3: Compressed on disk.", 0, 3);

    env->settings->createSetting("Cplex.NumericalEmphasis", "Subsolver", 0, "Emphasis on numerical stability", 0, 1);

    env->settings->createSetting("Cplex.ParallelMode", "Subsolver", 0,
        "Sets the parallel optimization mode: -1: Opportunistic. 0: Automatic. 1: Deterministic.", -1, 1);

    env->settings->createSetting("Cplex.Probe", "Subsolver", 0,
        "Sets the MIP probing level: -1: No probing. 0: Automatic. 1: Moderate. 2: Aggressive. 3: Very aggressive", -1,
        3);

    env->settings->createSetting("Cplex.SolutionPoolGap", "Subsolver", 1.0e+75,
        "Sets the relative gap filter on objective values in the solution pool", 0, 1.0e+75);

    env->settings->createSetting("Cplex.SolutionPoolIntensity", "Subsolver", 0,
        "Controls how much time and memory should be used when filling the solution pool: 0: Automatic. 1: Mild. "
        "2: "
        "Moderate. 3: Aggressive. 4: Very aggressive",
        0, 4);

    env->settings->createSetting("Cplex.SolutionPoolReplace", "Subsolver", 1,
        "How to replace solutions in the solution pool when full: 0: Replace oldest. 1: Replace worst. 2: Find "
        "diverse.",
        0, 2);

    env->settings->createSetting("Cplex.UseGenericCallback", "Subsolver", false,
        "Use the new generic callback (vers. >12.8) in the single-tree strategy (experimental)");

    std::string workdir = "";
    env->settings->createSetting("Cplex.WorkDirectory", "Subsolver", workdir, "Directory for swap file");

    env->settings->createSetting(
        "Cplex.WorkMemory", "Subsolver", 0.0, "Memory limit for when to start swapping to disk", 0.0, 1.0e+75);

#endif

    // Subsolver settings: Gurobi

#ifdef HAS_GUROBI

    env->settings->createSettingGroup("Subsolver", "Gurobi", "Gurobi", "");

    env->settings->createSetting(
        "Gurobi.ScaleFlag", "Subsolver", 0, "Controls model scaling: 0: Off. 1: Agressive. 2: Very agressive.", 0, 2);

    env->settings->createSetting("Gurobi.MIPFocus", "Subsolver", 0,
        "MIP focus: 0: Automatic. 1: Feasibility. 2: Optimality. 3: Best bound.", 0, 3);

    env->settings->createSetting("Gurobi.NumericFocus", "Subsolver", 0,
        "Numeric focus (higher number more careful): 0: Automatic. 3: Most careful.", 0, 3);

#endif

    // Subsolver settings: Cbc

#ifdef HAS_CBC

    env->settings->createSettingGroup("Subsolver", "Cbc", "Cbc", "");

    env->settings->createSetting("Cbc.AutoScale", "Subsolver", false,
        "Whether to scale objective, rhs and bounds of problem if they look odd (experimental)");

    env->settings->createSetting("Cbc.NodeStrategy", "Subsolver", std::string("fewest"),
        "Node strategy, valid values: depth, downdepth, downfewest, fewest, hybrid, updepth, upfewest");

    env->settings->createSetting(
        "Cbc.DeterministicParallelMode", "Subsolver", false, "Run Cbc with multiple threads in deterministic mode");

    env->settings->createSetting("Cbc.Scaling", "Subsolver", std::string("automatic"),
        "Whether to scale problem, valid values: automatic, dynamic, equilibrium, geometric, off, rowsonly");

    env->settings->createSetting("Cbc.Strategy", "Subsolver", 1,
        "This turns on newer features. Use 0 for easy problems, 1 is default, 2 is aggressive", 0, 2);

#endif

    // Subsolver settings: GAMS NLP

#ifdef HAS_GAMS

    env->settings->createSettingGroup("Subsolver", "GAMS", "GAMS", "Settings for the GAMS NLP solvers.");

    std::string optfile = "";
    env->settings->createSetting(
        "GAMS.NLP.OptionsFilename", "Subsolver", optfile, "Options file for the NLP solver in GAMS");

    std::string solver = "auto";
    env->settings->createSetting(
        "GAMS.NLP.Solver", "Subsolver", solver, "NLP solver to use in GAMS (auto: SHOT chooses)");

#endif

    // Subsolver settings: Ipopt

#ifdef HAS_IPOPT

    env->settings->createSettingGroup("Subsolver", "Ipopt", "Ipopt", "");

    env->settings->createSetting("Ipopt.ConstraintViolationTolerance", "Subsolver", 1E-8,
        "Constraint violation tolerance in Ipopt", SHOT_DBL_MIN, SHOT_DBL_MAX);

    VectorString enumIPOptSolver;
    enumIPOptSolver.push_back("Default");
    enumIPOptSolver.push_back("MA27");
    enumIPOptSolver.push_back("MA57");
    enumIPOptSolver.push_back("MA86");
    enumIPOptSolver.push_back("MA97");
    enumIPOptSolver.push_back("MUMPS");
    env->settings->createSetting("Ipopt.LinearSolver", "Subsolver", static_cast<int>(ES_IpoptSolver::IpoptDefault),
        "Ipopt linear subsolver", enumIPOptSolver);
    enumIPOptSolver.clear();

    env->settings->createSetting("Ipopt.MaxIterations", "Subsolver", 1000, "Maximum number of iterations");

    env->settings->createSetting(
        "Ipopt.RelativeConvergenceTolerance", "Subsolver", 1E-8, "Relative convergence tolerance");

#endif

    // Subsolver settings: root searches

    env->settings->createSettingGroup(
        "Subsolver", "Rootsearch", "Root search solver", "Settings for the Boost rootsearch functionality.");

    env->settings->createSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver", 0.0,
        "Epsilon constraint tolerance for root search", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "Rootsearch.MaxIterations", "Subsolver", 100, "Maximal root search iterations", 0, SHOT_INT_MAX);

    VectorString enumRootsearchMethod;
    enumRootsearchMethod.push_back("BoostTOMS748");
    enumRootsearchMethod.push_back("BoostBisection");
    enumRootsearchMethod.push_back("Bisection");
    env->settings->createSetting("Rootsearch.Method", "Subsolver", static_cast<int>(ES_RootsearchMethod::BoostTOMS748),
        "Root search method to use", enumRootsearchMethod);
    enumRootsearchMethod.clear();

    env->settings->createSetting("Rootsearch.TerminationTolerance", "Subsolver", 1e-16,
        "Epsilon lambda tolerance for root search", 0.0, SHOT_DBL_MAX);

    // Termination settings

    env->settings->createSettingGroup(
        "Termination", "", "Termination", "These settings control when SHOT will terminate the solution process.");

    env->settings->createSetting(
        "ConstraintTolerance", "Termination", 1e-8, "Termination tolerance for nonlinear constraints", 0, SHOT_DBL_MAX);

    env->settings->createSetting("ObjectiveConstraintTolerance", "Termination", 1e-8,
        "Termination tolerance for the nonlinear objective constraint", 0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "IterationLimit", "Termination", 200000, "Iteration limit for main strategy", 1, SHOT_INT_MAX);

    env->settings->createSetting("ObjectiveGap.Absolute", "Termination", 0.001,
        "Absolute gap termination tolerance for objective function", 0, SHOT_DBL_MAX);

    env->settings->createSetting("ObjectiveGap.Relative", "Termination", 0.001,
        "Relative gap termination tolerance for objective function", 0, SHOT_DBL_MAX);

    env->settings->createSetting("DualStagnation.ConstraintTolerance", "Termination", 1e-6,
        "Min absolute difference between max nonlinear constraint errors in subsequent iterations for termination", 0,
        SHOT_DBL_MAX);

    env->settings->createSetting("DualStagnation.IterationLimit", "Termination", 50,
        "Max number of iterations without significant dual objective value improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("PrimalStagnation.IterationLimit", "Termination", 50,
        "Max number of iterations without significant primal objective value improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "TimeLimit", "Termination", SHOT_DBL_MAX, "Time limit (s) for solver", 0.0, SHOT_DBL_MAX);

    // Hidden settings for problem information

    VectorString enumFileFormat;
    enumFileFormat.push_back("OSiL");
    enumFileFormat.push_back("GAMS");
    enumFileFormat.push_back("NL");
    enumFileFormat.push_back("None");
    env->settings->createSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::None),
        "The format of the problem file", enumFileFormat, true);
    enumFileFormat.clear();

    env->settings->createSetting("ProblemFile", "Input", empty, "The filename of the problem", true);

    env->settings->createSetting("ProblemName", "Input", empty, "The name of the problem instance", true);

    env->settings->createSetting("OptionsFile", "Input", empty, "The name of the options file used", true);

    env->settings->createSetting("ResultPath", "Output", empty, "The path where to save the result information", true);

    env->settings->settingsInitialized = true;

    env->output->outputDebug("Initialization of settings complete.");
}

void Solver::initializeDebugMode()
{
    auto debugPath = env->settings->getSetting<std::string>("Debug.Path", "Output");
    fs::filesystem::path debugDir(debugPath);

    if(fs::filesystem::exists(debugDir))
    {
        env->output->outputDebug("Debug directory " + debugPath + " already exists.");
    }
    else
    {
        if(fs::filesystem::create_directories(debugDir))
        {
            env->output->outputDebug("Debug directory " + debugPath + " created.");
        }
        else
        {
            env->output->outputWarning("Could not create debug directory.");
        }
    }

    fs::filesystem::path source(
        fs::filesystem::canonical(env->settings->getSetting<std::string>("ProblemFile", "Input")));

    fs::filesystem::copy_file(
        source.string(), (debugDir / source.filename()).string(), fs::filesystem::copy_options::overwrite_existing);
}

void Solver::verifySettings()
{
    env->output->setLogLevels(static_cast<E_LogLevel>(env->settings->getSetting<int>("Console.LogLevel", "Output")),
        static_cast<E_LogLevel>(env->settings->getSetting<int>("File.LogLevel", "Output")));

    // Checking for errors in NLP solver selection

    bool NLPSolverDefined = true;

#ifndef HAS_IPOPT
    if(static_cast<ES_PrimalNLPSolver>(env->settings->getSetting<int>("FixedInteger.Solver", "Primal"))
        == ES_PrimalNLPSolver::Ipopt)
    {
        env->output->outputWarning(" SHOT has not been compiled with support for Ipopt NLP solver.");
        NLPSolverDefined = false;
    }
#endif

#ifndef HAS_GAMS
    if(static_cast<ES_PrimalNLPSolver>(env->settings->getSetting<int>("FixedInteger.Solver", "Primal"))
        == ES_PrimalNLPSolver::GAMS)
    {
        env->output->outputWarning(" SHOT has not been compiled with support for GAMS NLP solvers.");
        NLPSolverDefined = false;
    }
#endif

#ifdef HAS_GAMS
    if((static_cast<ES_PrimalNLPSolver>(env->settings->getSetting<int>("FixedInteger.Solver", "Primal"))
           == ES_PrimalNLPSolver::GAMS)
        && (static_cast<ES_PrimalNLPProblemSource>(
                env->settings->getSetting<int>("FixedInteger.SourceProblem", "Primal"))
               != ES_PrimalNLPProblemSource::OriginalProblem))
    {
        env->output->outputWarning(" Cannot use GAMS NLP solvers when solving fixed NLP problems based on the "
                                   "reformulated model. Use Ipopt instead!");
        env->settings->updateSetting(
            "FixedInteger.SourceProblem", "Primal", static_cast<int>(ES_PrimalNLPProblemSource::OriginalProblem));
    }
#endif

    if((env->settings->getSetting<int>("SourceFormat", "Input") == static_cast<int>(ES_SourceFormat::OSiL)
           || env->settings->getSetting<int>("SourceFormat", "Input") == static_cast<int>(ES_SourceFormat::NL))
        && static_cast<ES_PrimalNLPSolver>(env->settings->getSetting<int>("FixedInteger.Solver", "Primal"))
            == ES_PrimalNLPSolver::GAMS)
    {
        env->output->outputWarning(" Cannot use GAMS NLP solvers with problem files in OSiL or nl formats.");
        NLPSolverDefined = false;
    }

    if(!NLPSolverDefined)
    {
#ifdef HAS_IPOPT
        env->settings->updateSetting("FixedInteger.Solver", "Primal", (int)ES_PrimalNLPSolver::Ipopt);
        env->output->outputWarning(" Using Ipopt as NLP solver instead.");

#elif HAS_GAMS
        env->settings->updateSetting("FixedInteger.Solver", "Primal", (int)ES_PrimalNLPSolver::GAMS);
        env->output->outputWarning(" Using GAMS NLP solvers instead.");

#else
        env->settings->updateSetting("FixedInteger.Use", "Primal", false);
        env->output->outputWarning(" No NLP solver available. Disabling primal strategy!");
#endif
    }

    // Checking for errors in MIP solver selection

    auto solver = static_cast<ES_MIPSolver>(env->settings->getSetting<int>("MIP.Solver", "Dual"));
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
        env->settings->updateSetting("TreeStrategy", "Dual", static_cast<int>(ES_TreeStrategy::MultiTree));
        env->settings->updateSetting(
            "Reformulation.Quadratics.Strategy", "Model", static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear));
    }
#endif

    if(!MIPSolverDefined)
    {
        env->output->outputWarning(" SHOT has not been compiled with support for selected MIP solver.");

#ifdef HAS_CBC
        env->settings->updateSetting("MIP.Solver", "Dual", (int)ES_MIPSolver::Cbc);
        unboundedVariableBound = 1e50;
#elif HAS_GUROBI
        env->settings->updateSetting("MIP.Solver", "Dual", (int)ES_MIPSolver::Gurobi);
        unboundedVariableBound = 1e20;
#elif HAS_CPLEX
        env->settings->updateSetting("MIP.Solver", "Dual", (int)ES_MIPSolver::Cplex);
        unboundedVariableBound = 1e20;
#else
        env->output->outputCritical(" SHOT has not been compiled with support for any MIP solver.");
#endif
    }

    // Updating max bound setting for unbounded variables
    double minLB = env->settings->getSetting<double>("Variables.Continuous.MinimumLowerBound", "Model");
    double maxUB = env->settings->getSetting<double>("Variables.Continuous.MaximumUpperBound", "Model");

    if(minLB < -unboundedVariableBound)
    {
        env->settings->updateSetting("Variables.Continuous.MinimumLowerBound", "Model", -unboundedVariableBound);
    }

    if(maxUB > unboundedVariableBound)
    {
        env->settings->updateSetting("Variables.Continuous.MaximumUpperBound", "Model", unboundedVariableBound);
    }

    // Checking for too tight termination criteria
    if(env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination") < 1e-8)
        (env->settings->updateSetting("ObjectiveGap.Relative", "Termination", 1e-10));

    if(env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") < 1e-8)
        (env->settings->updateSetting("ObjectiveGap.Absolute", "Termination", 1e-10));
}

void Solver::setConvexityBasedSettings()
{
    if(env->settings->getSetting<bool>("UseRecommendedSettings", "Strategy"))
    {
        if(env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex)
        {
            env->settings->updateSetting("ESH.InteriorPoint.CuttingPlane.IterationLimit", "Dual", 50);
            env->settings->updateSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual", 1);

            env->settings->updateSetting("ESH.Rootsearch.UniqueConstraints", "Dual", false);

            env->settings->updateSetting("HyperplaneCuts.ConstraintSelectionFactor", "Dual", 1.0);
            env->settings->updateSetting("HyperplaneCuts.UseIntegerCuts", "Dual", true);

            env->settings->updateSetting("TreeStrategy", "Dual", static_cast<int>(ES_TreeStrategy::MultiTree));

            env->settings->updateSetting("MIP.Presolve.UpdateObtainedBounds", "Dual", false);
            env->settings->updateSetting("MIP.SolutionLimit.Initial", "Dual", SHOT_INT_MAX);

            env->settings->updateSetting("Relaxation.Use", "Dual", false);

            env->settings->updateSetting(
                "Reformulation.Constraint.PartitionNonlinearTerms", "Model", (int)ES_PartitionNonlinearSums::Always);
            env->settings->updateSetting(
                "Reformulation.Constraint.PartitionQuadraticTerms", "Model", (int)ES_PartitionNonlinearSums::Always);
            env->settings->updateSetting("Reformulation.ObjectiveFunction.PartitionNonlinearTerms", "Model",
                (int)ES_PartitionNonlinearSums::Always);
            env->settings->updateSetting("Reformulation.ObjectiveFunction.PartitionQuadraticTerms", "Model",
                (int)ES_PartitionNonlinearSums::Always);
            // env->settings->updateSetting("Reformulation.Quadratics.Strategy", "Model", 0);

            env->settings->updateSetting("FixedInteger.CallStrategy", "Primal", 0);
            env->settings->updateSetting("FixedInteger.CreateInfeasibilityCut", "Primal", false);
            env->settings->updateSetting("FixedInteger.Source", "Primal", 0);
            env->settings->updateSetting("FixedInteger.Warmstart", "Primal", true);

            env->settings->updateSetting("FixedInteger.OnlyUniqueIntegerCombinations", "Primal", false);

            env->settings->updateSetting("Rootsearch.Use", "Primal", true);

            env->settings->updateSetting("BoundTightening.FeasibilityBased.TimeLimit", "Model", 5.0);

#ifdef HAS_CPLEX

            if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("MIP.Solver", "Dual")) == ES_MIPSolver::Cplex)
            {
                /*env->settings->updateSetting("Cplex.MIPEmphasis", "Subsolver", 4);
                env->settings->updateSetting("Cplex.NumericalEmphasis", "Subsolver", 1);
                env->settings->updateSetting("Cplex.Probe", "Subsolver", 3);
                env->settings->updateSetting("Cplex.SolutionPoolIntensity", "Subsolver", 4);
*/
                if(env->reformulatedProblem->objectiveFunction->properties.classification
                        == E_ObjectiveFunctionClassification::Quadratic
                    || env->reformulatedProblem->properties.numberOfQuadraticConstraints > 0)
                    env->settings->updateSetting("Cplex.OptimalityTarget", "Subsolver", 3);
            }

#endif
        }

#ifdef HAS_CBC
        if(static_cast<ES_MIPSolver>(env->settings->getSetting<int>("MIP.Solver", "Dual")) == ES_MIPSolver::Cbc)
        {
            env->settings->updateSetting(
                "Reformulation.Constraint.PartitionNonlinearTerms", "Model", (int)ES_PartitionNonlinearSums::IfConvex);
            env->settings->updateSetting("Reformulation.ObjectiveFunction.PartitionNonlinearTerms", "Model",
                (int)ES_PartitionNonlinearSums::IfConvex);
        }

#endif
    }
}

void Solver::updateSetting(std::string name, std::string category, int value)
{
    env->settings->updateSetting(name, category, value);
}

void Solver::updateSetting(std::string name, std::string category, std::string value)
{
    env->settings->updateSetting(name, category, value);
}

void Solver::updateSetting(std::string name, std::string category, double value)
{
    env->settings->updateSetting(name, category, value);
}

void Solver::updateSetting(std::string name, std::string category, bool value)
{
    env->settings->updateSetting(name, category, value);
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
} // namespace SHOT
