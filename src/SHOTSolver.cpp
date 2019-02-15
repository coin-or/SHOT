/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "SHOTSolver.h"

namespace SHOT
{

SHOTSolver::SHOTSolver(std::shared_ptr<spdlog::sinks::sink> consoleSink)
{
    env = std::make_shared<Environment>();

    env->output = std::make_shared<Output>();
    if(consoleSink != NULL)
        env->output->setConsoleSink(consoleSink);

    env->results = std::make_shared<Results>(env);
    env->timing = std::make_shared<Timing>(env);

    env->timing->createTimer("Total", "Total solution time");
    env->timing->startTimer("Total");

    env->timing->createTimer("ProblemInitialization", " - problem initialization");

    env->settings = std::make_shared<Settings>(env->output);
    env->tasks = std::make_shared<TaskHandler>(env);
    env->report = std::make_shared<Report>(env);

    env->dualSolver = std::make_shared<DualSolver>(env);
    env->primalSolver = std::make_shared<PrimalSolver>(env);
    initializeSettings();
}

SHOTSolver::SHOTSolver(EnvironmentPtr envPtr) : env(envPtr) { initializeSettings(); }

SHOTSolver::~SHOTSolver() {}

bool SHOTSolver::setOptions(std::string fileName)
{
    auto osolreader = std::make_unique<OSoLReader>();

    try
    {
        std::string fileContents;
        std::string fileExtension = boost::filesystem::extension(fileName);

        if(fileExtension == ".xml" || fileExtension == ".osol")
        {
            fileContents = UtilityFunctions::getFileAsString(fileName);

            // try
            {
                env->settings->readSettingsFromOSoL(fileContents);
            }
            /*catch(std::exception& e)
            {
                env->output->outputError("Error when reading OSoL options file " + fileName, e.what());
                return (false);
            }*/

            verifySettings();
        }
        else if(fileExtension == ".opt")
        {
            fileContents = UtilityFunctions::getFileAsString(fileName);

            // try
            {
                env->settings->readSettingsFromString(fileContents);
            }
            /*catch(std::exception& e)
            {
                env->output->outputError("Error when reading options file" + fileName);
                return (false);
            }*/
        }
        else
        {
            env->output->outputError(
                "Error when reading options from \"" + fileName + "\". File extension must be osol, xml or opt.");
        }
    }
    catch(const ErrorClass& eclass)
    {
        env->output->outputError("Error when reading options from \"" + fileName + "\"", eclass.errormsg);
        return (false);
    }

    env->settings->updateSetting("OptionsFile", "Input", fileName);

    env->output->outputDebug("Options read from file \"" + fileName + "\"");

    return (true);
}

bool SHOTSolver::setProblem(std::string fileName)
{
    if(!boost::filesystem::exists(fileName))
    {
        env->output->outputError("Problem file \"" + fileName + "\" does not exist.");

        return (false);
    }

    boost::filesystem::path problemFile(fileName);

    if(!problemFile.has_extension())
    {
        env->output->outputError("Problem file \"" + fileName + "\" does not specify a file extension.");

        return (false);
    }

    boost::filesystem::path problemExtension = problemFile.extension();
    boost::filesystem::path problemPath = problemFile.parent_path();

    try
    {
        if(problemExtension == ".osil" || problemExtension == ".xml")
        {
            auto modelingSystem = std::make_shared<ModelingSystemOS>(env);
            ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

            if(modelingSystem->createProblem(problem, fileName, E_OSInputFileFormat::OSiL)
                != E_ProblemCreationStatus::NormalCompletion)
            {
                return (false);
            }

            env->modelingSystem = modelingSystem;
            env->problem = problem;

            env->reformulatedProblem = problem;

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::OSiL));

            if(static_cast<ES_PrimalNLPSolver>(env->settings->getIntSetting("FixedInteger.Solver", "Primal"))
                == ES_PrimalNLPSolver::GAMS)
            {
                env->output->outputError(
                    "Cannot use GAMS NLP solvers in combination with OSiL-files. Switching to Ipopt");
                env->settings->updateSetting("FixedInteger.Solver", "Primal", (int)ES_PrimalNLPSolver::Ipopt);
            }
        }
        else if(problemExtension == ".nl")
        {
            auto modelingSystem = std::make_shared<ModelingSystemOS>(env);
            ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

            if(modelingSystem->createProblem(problem, fileName, E_OSInputFileFormat::Ampl)
                != E_ProblemCreationStatus::NormalCompletion)
            {
            }
            else
            {
            }

            env->modelingSystem = modelingSystem;
            env->problem = problem;
            env->reformulatedProblem = problem;

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::NL));
        }
#ifdef HAS_GAMS
        else if(problemExtension == ".gms")
        {
            auto modelingSystem = std::make_shared<SHOT::ModelingSystemGAMS>(env);
            SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

            if(modelingSystem->createProblem(problem, fileName, E_GAMSInputSource::ProblemFile)
                != E_ProblemCreationStatus::NormalCompletion)
            {
                std::cout << "Error while reading problem";
            }
            else
            {
            }

            env->modelingSystem = modelingSystem;
            env->problem = problem;
            env->reformulatedProblem = problem;

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::GAMS));
        }
        else if(problemExtension == ".dat")
        {
            auto modelingSystem = std::make_shared<SHOT::ModelingSystemGAMS>(env);
            SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

            if(modelingSystem->createProblem(problem, fileName, E_GAMSInputSource::GAMSModel)
                != E_ProblemCreationStatus::NormalCompletion)
            {
                std::cout << "Error while reading problem";
            }
            else
            {
            }

            env->modelingSystem = modelingSystem;
            env->problem = problem;
            env->reformulatedProblem = problem;

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::GAMS));
        }
#endif
        else
        {
            env->output->outputError("Wrong filetype specified.");

            return (false);
        }

        if(env->settings->getBoolSetting("Debug.Enable", "Output"))
        {
            std::stringstream problemFilename;
            problemFilename << env->settings->getStringSetting("Debug.Path", "Output");
            problemFilename << "/originalproblem.txt";

            std::stringstream problemText;
            problemText << env->reformulatedProblem;

            UtilityFunctions::writeStringToFile(problemFilename.str(), problemText.str());
        }
    }
    catch(const ErrorClass& eclass)
    {
        env->output->outputError("Error when reading problem from \"" + fileName + "\"", eclass.errormsg);

        return (false);
    }

    env->settings->updateSetting("ProblemFile", "Input", problemFile.string());

    // Removes path
    boost::filesystem::path problemName = problemFile.stem();
    env->settings->updateSetting("ProblemName", "Input", problemName.string());

    if(static_cast<ES_OutputDirectory>(env->settings->getIntSetting("OutputDirectory", "Output"))
        == ES_OutputDirectory::Program)
    {
        boost::filesystem::path debugPath(boost::filesystem::current_path());
        debugPath /= problemName;

        env->settings->updateSetting("Debug.Path", "Output", "problemdebug/" + problemName.string());
        env->settings->updateSetting("ResultPath", "Output", boost::filesystem::current_path().string());
    }
    else
    {
        boost::filesystem::path debugPath(problemPath);
        debugPath /= problemName;

        env->settings->updateSetting("Debug.Path", "Output", debugPath.string());
        env->settings->updateSetting("ResultPath", "Output", problemPath.string());
    }

    if(env->settings->getBoolSetting("Debug.Enable", "Output"))
    {
        initializeDebugMode();

        std::stringstream filename;
        filename << env->settings->getStringSetting("Debug.Path", "Output");
        filename << "/originalproblem";
        filename << ".txt";

        std::stringstream problem;
        problem << env->problem;

        UtilityFunctions::writeStringToFile(filename.str(), problem.str());
    }

    verifySettings();

    bool status = this->selectStrategy();

    return (true);
}

bool SHOTSolver::setProblem(SHOT::ProblemPtr problem, SHOT::ModelingSystemPtr modelingSystem)
{
    env->modelingSystem = modelingSystem;
    env->problem = problem;
    env->reformulatedProblem = problem;

    env->settings->updateSetting("ProblemName", "Input", problem->name);

    if(static_cast<ES_OutputDirectory>(env->settings->getIntSetting("OutputDirectory", "Output"))
        == ES_OutputDirectory::Program)
    {
        boost::filesystem::path debugPath(boost::filesystem::current_path());
        debugPath /= problem->name;

        env->settings->updateSetting("Debug.Path", "Output", "problemdebug/" + problem->name);
        env->settings->updateSetting("ResultPath", "Output", boost::filesystem::current_path().string());
    }

    if(env->settings->getBoolSetting("Debug.Enable", "Output"))
    {
        initializeDebugMode();

        std::stringstream filename;
        filename << env->settings->getStringSetting("Debug.Path", "Output");
        filename << "/originalproblem";
        filename << ".txt";

        std::stringstream problem;
        problem << env->problem;

        UtilityFunctions::writeStringToFile(filename.str(), problem.str());
    }

    verifySettings();

    this->selectStrategy();

    return (true);
}

bool SHOTSolver::selectStrategy()
{
    if(static_cast<ES_MIPSolver>(env->settings->getIntSetting("MIP.Solver", "Dual")) == ES_MIPSolver::Cbc)
    {
        if(env->problem->properties.isNLPProblem)
        {
            env->output->outputDebug(" Using NLP solution strategy.");
            solutionStrategy = std::make_unique<SolutionStrategyNLP>(env);

            env->results->usedSolutionStrategy = E_SolutionStrategy::NLP;
        }
        else
        {
            solutionStrategy = std::make_unique<SolutionStrategyMultiTree>(env);
            isProblemInitialized = true;
        }

        return (true);
    }

    auto quadraticStrategy = static_cast<ES_QuadraticProblemStrategy>(
        env->settings->getIntSetting("Reformulation.Quadratics.Strategy", "Model"));
    bool useQuadraticConstraints = (quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticallyConstrained);
    bool useQuadraticObjective
        = (useQuadraticConstraints || quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticObjective);

    if((useQuadraticObjective || useQuadraticConstraints) && env->problem->properties.isMIQPProblem)
    // MIQP problem
    {
        env->output->outputDebug(" Using MIQP solution strategy.");
        solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
        env->results->usedSolutionStrategy = E_SolutionStrategy::MIQP;
    }
    // MIQCQP problem
    else if(useQuadraticConstraints && env->problem->properties.isMIQCQPProblem)
    {
        env->output->outputDebug(" Using MIQCQP solution strategy.");

        solutionStrategy = std::make_unique<SolutionStrategyMIQCQP>(env);
        env->results->usedSolutionStrategy = E_SolutionStrategy::MIQCQP;
    }
    // NLP problem
    else if(env->problem->properties.isNLPProblem)
    {
        env->output->outputDebug(" Using NLP solution strategy.");
        solutionStrategy = std::make_unique<SolutionStrategyNLP>(env);
        env->results->usedSolutionStrategy = E_SolutionStrategy::NLP;
    }
    else
    {
        switch(static_cast<ES_TreeStrategy>(env->settings->getIntSetting("TreeStrategy", "Dual")))
        {
        case(ES_TreeStrategy::SingleTree):
            env->output->outputDebug(" Using single-tree solution strategy.");
            solutionStrategy = std::make_unique<SolutionStrategySingleTree>(env);
            env->results->usedSolutionStrategy = E_SolutionStrategy::SingleTree;
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

    isProblemInitialized = true;

    return (true);
}

bool SHOTSolver::solveProblem()
{
    if(env->settings->getBoolSetting("Debug.Enable", "Output"))
    {
        std::stringstream filename;
        filename << env->settings->getStringSetting("Debug.Path", "Output");
        filename << "/usedsettings";
        filename << ".opt";

        auto usedSettings = env->settings->getSettingsAsString(false, false);

        UtilityFunctions::writeStringToFile(filename.str(), usedSettings);
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

    bool result = solutionStrategy->solveProblem();

    if(result)
        isProblemSolved = true;

    return (result);
}

std::string SHOTSolver::getResultsOSrL() { return (env->results->getResultsOSrL()); }

std::string SHOTSolver::getOptionsOSoL()
{
    if(!env->settings->settingsInitialized)
        initializeSettings();

    return (env->settings->getSettingsAsOSoL());
}

std::string SHOTSolver::getOptions()
{
    if(!env->settings->settingsInitialized)
        initializeSettings();

    return (env->settings->getSettingsAsString(false, false));
}

std::string SHOTSolver::getResultsTrace() { return (env->results->getResultsTrace()); }

void SHOTSolver::initializeSettings()
{
    if(env->settings->settingsInitialized)
    {
        env->output->outputWarning("Warning! Settings have already been initialized. Ignoring new settings.");
        return;
    }

    std::string empty; // Used to create empty string options

    env->output->outputDebug("Starting initialization of settings:");

    // Convexity strategy

    env->settings->createSetting("UseRecommendedSettings", "Strategy", true,
        "Modifies some settings to their recommended values based on the strategy");

    VectorString enumConvexityIdentificationStrategy;
    enumConvexityIdentificationStrategy.push_back("Automatically");
    enumConvexityIdentificationStrategy.push_back("AssumeConvex");
    enumConvexityIdentificationStrategy.push_back("AssumeNonconvex");
    env->settings->createSetting("Convexity", "Strategy",
        static_cast<int>(ES_ConvexityIdentificationStrategy::Automatically),
        "How to determine convexity of the problem", enumConvexityIdentificationStrategy);
    enumConvexityIdentificationStrategy.clear();

    // Dual strategy settings: ECP and ESH

    VectorString enumHyperplanePointStrategy;
    enumHyperplanePointStrategy.push_back("ESH");
    enumHyperplanePointStrategy.push_back("ECP");
    env->settings->createSetting("CutStrategy", "Dual", static_cast<int>(ES_HyperplaneCutStrategy::ESH),
        "Dual cut strategy", enumHyperplanePointStrategy);
    enumHyperplanePointStrategy.clear();

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.BitPrecision", "Dual", 8,
        "Required termination bit precision for minimization subsolver", 1, 64, true);

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.ConstraintSelectionFactor", "Dual", 0.25,
        "The fraction of violated constraints to generate cutting planes for", 0.0, 1.0);

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.IterationLimit", "Dual", 100,
        "Iteration limit for minimax cutting plane solver", 1, SHOT_INT_MAX);

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.IterationLimitSubsolver", "Dual", 100,
        "Iteration limit for minimization subsolver", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "ESH.InteriorPoint.CuttingPlane.Reuse", "Dual", true, "Reuse valid cutting planes in main dual model");

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceAbs", "Dual", 1.0,
        "Absolute termination tolerance between LP and linesearch objective", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceRel", "Dual", 1.0,
        "Relative termination tolerance between LP and linesearch objective", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("ESH.InteriorPoint.MinimaxObjectiveLowerBound", "Dual", -999999999999.0,
        "Lower bound for minimax objective variable", SHOT_DBL_MIN, 0);

    env->settings->createSetting("ESH.InteriorPoint.MinimaxObjectiveUpperBound", "Dual", 0.1,
        "Upper bound for minimax objective variable", SHOT_DBL_MIN, SHOT_DBL_MAX);

    // Dual strategy settings: Interior point search strategy

    VectorString enumNLPSolver;
    enumNLPSolver.push_back("Cutting plane minimax");
    enumNLPSolver.push_back("Ipopt minimax");
    enumNLPSolver.push_back("Ipopt relaxed");
    enumNLPSolver.push_back("Ipopt minimax and relaxed");

    env->settings->createSetting("ESH.InteriorPoint.Solver", "Dual",
        static_cast<int>(ES_InteriorPointStrategy::CuttingPlaneMiniMax), "NLP solver", enumNLPSolver, true);
    enumNLPSolver.clear();

    VectorString enumAddPrimalPointAsInteriorPoint;
    enumAddPrimalPointAsInteriorPoint.push_back("No");
    enumAddPrimalPointAsInteriorPoint.push_back("Add as new");
    enumAddPrimalPointAsInteriorPoint.push_back("Replace old");
    enumAddPrimalPointAsInteriorPoint.push_back("Use avarage");
    env->settings->createSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual",
        static_cast<int>(ES_AddPrimalPointAsInteriorPoint::OnlyAverage), "Utilize primal solution as interior point",
        enumAddPrimalPointAsInteriorPoint);
    enumAddPrimalPointAsInteriorPoint.clear();

    env->settings->createSetting("HyperplaneCuts.MaxConstraintFactor", "Dual", 0.5,
        "Linesearch performed on constraints with values larger than this factor times the maximum value", 1e-6, 1.0);

    env->settings->createSetting(
        "ESH.Linesearch.UniqueConstraints", "Dual", false, "Allow only one hyperplane per constraint per iteration");

    env->settings->createSetting("ESH.Linesearch.ConstraintTolerance", "Dual", 1e-8,
        "Constraint tolerance for when not to add individual hyperplanes", 0, SHOT_DBL_MAX);

    // Dual strategy settings: Fixed integer (NLP) strategy

    env->settings->createSetting("FixedInteger.ConstraintTolerance", "Dual", 0.0001,
        "Constraint tolerance for fixed strategy", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "FixedInteger.MaxIterations", "Dual", 20, "Max LP iterations for fixed strategy", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "FixedInteger.ObjectiveTolerance", "Dual", 0.001, "Objective tolerance for fixed strategy", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("FixedInteger.Use", "Dual", false,
        "Solve a fixed LP problem if integer-values have not changes in several MIP iterations");

    // Dual strategy settings: Hyperplane generation

    env->settings->createSetting("HyperplaneCuts.ConstraintSelectionFactor", "Dual", 0.5,
        "The fraction of violated constraints to generate supporting hyperplanes / cutting planes for", 0.0, 1.0);

    env->settings->createSetting(
        "HyperplaneCuts.Delay", "Dual", true, "Add hyperplane cuts to model only after optimal MIP solution");

    env->settings->createSetting("HyperplaneCuts.MaxPerIteration", "Dual", 200,
        "Maximal number of hyperplanes to add per iteration", 0, SHOT_INT_MAX);

    env->settings->createSetting("HyperplaneCuts.UseIntegerCuts", "Dual", true,
        "Add integer cuts for infeasible integer-combinations for binary problems");

    env->settings->createSetting(
        "HyperplaneCuts.UsePrimalObjectiveCut", "Dual", true, "Add an objective cut in the primal solution");

    // Dual strategy settings: MIP solver

    env->settings->createSetting("MIP.CutOffTolerance", "Dual", 0.00001,
        "An extra tolerance for the objective cutoff value (to prevent infeasible subproblems)", SHOT_DBL_MIN,
        SHOT_DBL_MAX);

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

    env->settings->createSetting(
        "MIP.NumberOfThreads", "Dual", 8, "Number of threads to use in MIP solver: 0: Automatic", 0, 999);

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
        "MIP.Solver", "Dual", static_cast<int>(ES_MIPSolver::Cplex), "What MIP solver to use", enumMIPSolver);
    enumMIPSolver.clear();

    env->settings->createSetting(
        "MIP.UpdateObjectiveBounds", "Dual", false, "Update nonlinear objective variable bounds to primal/dual bounds");

    // Dual strategy settings: Relaxation strategies

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

    VectorString enumSolutionStrategy;
    enumSolutionStrategy.push_back("Multi-tree");
    enumSolutionStrategy.push_back("Single-tree");
    env->settings->createSetting("TreeStrategy", "Dual", static_cast<int>(ES_TreeStrategy::SingleTree),
        "The main strategy to use", enumSolutionStrategy);
    enumSolutionStrategy.clear();

    env->settings->createSetting("TreeStrategy.Multi.Reinitialize", "Dual", false,
        "Reinitialize the dual model in the subsolver each iteration");

    // Optimization model settings
    env->settings->createSetting("ContinuousVariable.EmptyLowerBound", "Model", -9999999999.0,
        "Lower bound for continuous variables without bounds", 0, SHOT_DBL_MAX);

    env->settings->createSetting("ContinuousVariable.EmptyUpperBound", "Model", 9999999999.0,
        "Upper bound for continuous variables without bounds", 0, SHOT_DBL_MAX);

    env->settings->createSetting("IntegerVariable.EmptyLowerBound", "Model", 0.0,
        "Lower bound for integer variables without bounds", 0, SHOT_DBL_MAX);

    env->settings->createSetting("IntegerVariable.EmptyUpperBound", "Model", 2.0e9,
        "Upper bound for integer variables without bounds", 0, SHOT_DBL_MAX);

    env->settings->createSetting("NonlinearObjectiveVariable.Bound", "Model", 999999999999.0,
        "Max absolute bound for the auxiliary nonlinear objective variable", 0, SHOT_DBL_MAX);

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

    // Reformulations for constraints
    env->settings->createSetting("Reformulation.Constraint.PartitionNonlinearTerms", "Model", false,
        "Partition nonlinear terms as auxiliary constraints");

    env->settings->createSetting("Reformulation.Constraint.PartitionQuadraticTerms", "Model", false,
        "Partition quadratic terms as auxiliary constraints");

    // Reformulations for binary monomials
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
        "Reformulates a nonlinear objective as an auxilliary constraint");

    env->settings->createSetting("Reformulation.ObjectiveFunction.PartitionNonlinearTerms", "Model", false,
        "Partition nonlinear terms as auxilliary constraints");

    env->settings->createSetting("Reformulation.ObjectiveFunction.PartitionQuadraticTerms", "Model", false,
        "Partition quadratic terms as auxilliary constraints");

    // Reformulations for quadratic objective and constraints
    VectorString enumQPStrategy;
    enumQPStrategy.push_back("All nonlinear");
    enumQPStrategy.push_back("Use quadratic objective");
    enumQPStrategy.push_back("Use quadratic objective and constraints");
    env->settings->createSetting("Reformulation.Quadratics.Strategy", "Model",
        static_cast<int>(ES_QuadraticProblemStrategy::QuadraticObjective), "How to treat quadratic functions",
        enumQPStrategy);
    enumQPStrategy.clear();

    // Logging and output settings
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

    env->settings->createSetting("Console.GAMS.Show", "Output", false, "Show GAMS output on console");

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

    VectorString enumPrimalNLPSolver;
    enumPrimalNLPSolver.push_back("CuttingPlane");
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

    env->settings->createSetting(
        "FixedInteger.TimeLimit", "Primal", 10.0, "Time limit (s) per NLP problem", 0, SHOT_DBL_MAX);

    env->settings->createSetting("FixedInteger.Use", "Primal", true, "Use the fixed integer primal strategy");

    env->settings->createSetting("FixedInteger.UsePresolveBounds", "Primal", false,
        "Use variable bounds from MIP in NLP problems. Warning! Does not seem to work", true);

    env->settings->createSetting("FixedInteger.Warmstart", "Primal", true, "Warm start the NLP solver");

    // Primal settings: linesearch

    env->settings->createSetting("Linesearch.Use", "Primal", true, "Use a linesearch to find primal solutions");

    // Primal settings: tolerances for accepting primal solutions

    env->settings->createSetting("Tolerance.TrustLinearConstraintValues", "Primal", true,
        "Trust that subsolvers (NLP, MIP) give primal solutions that respect linear constraints");

    env->settings->createSetting(
        "Tolerance.Integer", "Primal", 1e-5, "Integer tolerance for accepting primal solutions");

    env->settings->createSetting(
        "Tolerance.LinearConstraint", "Primal", 1e-6, "Linear constraint tolerance for accepting primal solutions");

    env->settings->createSetting("Tolerance.NonlinearConstraint", "Primal", 1e-6,
        "Nonlinear constraint tolerance for accepting primal solutions");

    // Subsolver settings: Cplex

    env->settings->createSetting("Cplex.AddRelaxedLazyConstraintsAsLocal", "Subsolver", false,
        "Whether to add lazy constraints generated in relaxed points as local or global");

    env->settings->createSetting(
        "Cplex.OptimalityTarget", "Subsolver", 0, "Specifies how CPLEX treats nonconvex quadratics", 0, 3);

    env->settings->createSetting(
        "Cplex.FeasOptMode", "Subsolver", 0, "Strategy to use for the feasibility repair", 0, 5);

    env->settings->createSetting("Cplex.MemoryEmphasis", "Subsolver", 0, "Try to conserve memory when possible", 0, 1);

    env->settings->createSetting("Cplex.MIPEmphasis", "Subsolver", 0,
        "Sets the MIP emphasis: 0: Balanced. 1: Feasibility. 2: Optimality. 3: Best bound. 4: Hidden feasible", 0, 4);

    env->settings->createSetting("Cplex.NodeFileInd", "Subsolver", 1,
        "Where to store the node file: 0: No file. 1: Compressed in memory. 2: On disk. 3: Compressed on disk.", 0, 3);

    env->settings->createSetting("Cplex.NumericalEmphasis", "Subsolver", 0, "Emphasis on numerical stability", 0, 1);

    env->settings->createSetting("Cplex.ParallelMode", "Subsolver", 0,
        "Sets the parallel optimization mode: -1: Opportunistic. 0: Automatic. 1: Deterministic.", -1, 1);

    env->settings->createSetting("Cplex.Probe", "Subsolver", 0,
        "Sets the MIP probing level: -1: No probing. 0: Automatic. 1: Moderate. 2: Aggressive. 3: Very aggressive", -1,
        3);

    env->settings->createSetting("Cplex.SolnPoolGap", "Subsolver", 1.0e+75,
        "Sets the relative gap filter on objective values in the solution pool", 0, 1.0e+75);

    env->settings->createSetting("Cplex.SolnPoolIntensity", "Subsolver", 0,
        "Controls how much time and memory should be used when filling the solution pool: 0: Automatic. 1: Mild. 2: "
        "Moderate. 3: Aggressive. 4: Very aggressive",
        0, 4);

    env->settings->createSetting("Cplex.SolnPoolReplace", "Subsolver", 1,
        "How to replace solutions in the solution pool when full: 0: Replace oldest. 1: Replace worst. 2: Find "
        "diverse.",
        0, 2);

    env->settings->createSetting("Cplex.UseNewCallbackType", "Subsolver", false,
        "Use the new callback type (vers. >12.8) with single-tree strategy (experimental)");

    std::string workdir = "/data/stuff/tmp/";
    env->settings->createSetting("Cplex.WorkDir", "Subsolver", workdir, "Directory for swap file");

    env->settings->createSetting(
        "Cplex.WorkMem", "Subsolver", 30000.0, "Memory limit for when to start swapping to disk", 0, 1.0e+75);

    // Subsolver settings: Gurobi
    env->settings->createSetting(
        "Gurobi.ScaleFlag", "Subsolver", 1, "Controls model scaling: 0: Off. 1: Agressive. 2: Very agressive.", 0, 2);

    env->settings->createSetting("Gurobi.MIPFocus", "Subsolver", 0,
        "MIP focus: 0: Automatic. 1: Feasibility. 2: Optimality. 3: Best bound.", 0, 3);

    env->settings->createSetting("Gurobi.NumericFocus", "Subsolver", 0,
        "Numeric focus (higher number more careful): 0: Automatic. 3: Most careful.", 0, 3);

    // Subsolver settings: GAMS NLP

    std::string optfile = "";
    env->settings->createSetting(
        "GAMS.NLP.OptionsFilename", "Subsolver", optfile, "Options file for the NLP solver in GAMS");

    std::string solver = "conopt";
    env->settings->createSetting("GAMS.NLP.Solver", "Subsolver", solver, "NLP solver to use in GAMS");

    // Subsolver settings: Ipopt

    env->settings->createSetting("Ipopt.ConstraintViolationTolerance", "Subsolver", 1E-8,
        "Constraint violation tolerance in Ipopt", SHOT_DBL_MIN, SHOT_DBL_MAX);

    VectorString enumIPOptSolver;
    enumIPOptSolver.push_back("ma27");
    enumIPOptSolver.push_back("ma57");
    enumIPOptSolver.push_back("ma86");
    enumIPOptSolver.push_back("ma97");
    enumIPOptSolver.push_back("mumps");
    env->settings->createSetting("Ipopt.LinearSolver", "Subsolver", static_cast<int>(ES_IpoptSolver::ma57),
        "Ipopt linear subsolver", enumIPOptSolver);
    enumIPOptSolver.clear();

    env->settings->createSetting("Ipopt.MaxIterations", "Subsolver", 1000, "Maximum number of iterations");

    env->settings->createSetting(
        "Ipopt.RelativeConvergenceTolerance", "Subsolver", 1E-8, "Relative convergence tolerance");

    // Subsolver settings: root searches

    env->settings->createSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver", 0.0,
        "Epsilon constraint tolerance for root search", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "Rootsearch.MaxIterations", "Subsolver", 100, "Maximal root search iterations", 0, SHOT_INT_MAX);

    VectorString enumLinesearchMethod;
    enumLinesearchMethod.push_back("BoostTOMS748");
    enumLinesearchMethod.push_back("BoostBisection");
    enumLinesearchMethod.push_back("Bisection");
    env->settings->createSetting("Rootsearch.Method", "Subsolver", static_cast<int>(ES_RootsearchMethod::BoostTOMS748),
        "Root search method to use", enumLinesearchMethod);
    enumLinesearchMethod.clear();

    env->settings->createSetting("Rootsearch.TerminationTolerance", "Subsolver", 1e-16,
        "Epsilon lambda tolerance for root search", 0.0, SHOT_DBL_MAX);

    // Subsolver settings: termination

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

    env->settings->createSetting("DualStagnation.IterationLimit", "Termination", 50,
        "Max number of iterations without significant dual objective value improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("PrimalStagnation.IterationLimit", "Termination", 50,
        "Max number of iterations without significant primal objective value improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("InfeasibilityRepair.IterationLimit", "Termination", 100,
        "Max number of infeasible problems repaired without primal objective value improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("InfeasibilityRepair.TimeLimit", "Termination", 10.0,
        "Time limit when reparing infeasible problem", 0, SHOT_DBL_MAX);

    env->settings->createSetting("PrimalStagnation.MaxNumberOfPrimalCutReduction", "Termination", 5,
        "Max number of primal cut reduction without primal improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("TimeLimit", "Termination", 900.0, "Time limit (s) for solver", 0.0, SHOT_DBL_MAX);

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

void SHOTSolver::initializeDebugMode()
{
    auto debugPath = env->settings->getStringSetting("Debug.Path", "Output");
    boost::filesystem::path debugDir(debugPath);

    if(boost::filesystem::exists(debugDir))
    {
        env->output->outputDebug("Debug directory " + debugPath + " already exists.");
    }
    else
    {
        if(boost::filesystem::create_directories(debugDir))
        {
            env->output->outputDebug("Debug directory " + debugPath + " created.");
        }
        else
        {
            env->output->outputWarning("Could not create debug directory.");
        }
    }

    boost::filesystem::path source(env->settings->getStringSetting("ProblemFile", "Input"));
    boost::filesystem::copy_file(boost::filesystem::canonical(source), debugDir / source.filename(),
        boost::filesystem::copy_option::overwrite_if_exists);
}

void SHOTSolver::verifySettings()
{
    if(env->settings->getIntSetting("SourceFormat", "Input") == static_cast<int>(ES_SourceFormat::GAMS))
    {
        if(static_cast<ES_PrimalNLPSolver>(env->settings->getIntSetting("FixedInteger.Solver", "Primal"))
            == ES_PrimalNLPSolver::Ipopt)
        {
            env->output->outputWarning(" Changing to GAMS NLP solver since problem is given in GAMS format.");
            env->settings->updateSetting("FixedInteger.Solver", "Primal", (int)ES_PrimalNLPSolver::GAMS);
        }
    }

    if(env->settings->getIntSetting("SourceFormat", "Input") == static_cast<int>(ES_SourceFormat::OSiL)
        || env->settings->getIntSetting("SourceFormat", "Input") == static_cast<int>(ES_SourceFormat::NL))
    {
        if(static_cast<ES_PrimalNLPSolver>(env->settings->getIntSetting("FixedInteger.Solver", "Primal"))
            == ES_PrimalNLPSolver::Ipopt)
        {
            env->output->outputWarning(" Using Ipopt as NLP solver since problem is given in OSiL or Ampl format.");
            env->settings->updateSetting("FixedInteger.Solver", "Primal", (int)ES_PrimalNLPSolver::Ipopt);
        }
    }

    if(static_cast<ES_PrimalNLPSolver>(env->settings->getIntSetting("FixedInteger.Solver", "Primal"))
        == ES_PrimalNLPSolver::GAMS)
    {
#ifndef HAS_GAMS
        env->output->outputWarning(" SHOT has not been compiled with support for GAMS NLP solvers.");
        env->settings->updateSetting("FixedInteger.Solver", "Primal", (int)ES_PrimalNLPSolver::Ipopt);
#endif
    }

    auto solver = static_cast<ES_MIPSolver>(env->settings->getIntSetting("MIP.Solver", "Dual"));
    bool correctSolverDefined = false;
#ifdef HAS_CPLEX
    if(solver == ES_MIPSolver::Cplex)
    {
        correctSolverDefined = true;
    }
#endif

#ifdef HAS_GUROBI
    if(solver == ES_MIPSolver::Gurobi)
    {
        correctSolverDefined = true;
    }
#endif

    if(solver == ES_MIPSolver::Cbc)
    {
        correctSolverDefined = true;
    }

    if(!correctSolverDefined)
    {
        env->output->outputWarning(" SHOT has not been compiled with support selected MIP solver. Defaulting to Cbc.");
        env->settings->updateSetting("MIP.Solver", "Dual", (int)ES_MIPSolver::Cbc);
    }

    if(env->settings->getBoolSetting("UseRecommendedSettings", "Strategy"))
    {

        switch(static_cast<ES_ConvexityIdentificationStrategy>(env->settings->getIntSetting("Convexity", "Strategy")))
        {
        case(ES_ConvexityIdentificationStrategy::Automatically):
            break;

        case(ES_ConvexityIdentificationStrategy::AssumeConvex):
            break;

        case(ES_ConvexityIdentificationStrategy::AssumeNonconvex):
            // env->settings->updateSetting("CutStrategy", "Dual", 1);
            env->settings->updateSetting("ESH.InteriorPoint.CuttingPlane.IterationLimit", "Dual", 50);
            env->settings->updateSetting("ESH.InteriorPoint.CuttingPlane.Reuse", "Dual", false);
            env->settings->updateSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual", 1);
            env->settings->updateSetting("MIP.Presolve.UpdateObtainedBounds", "Dual", false);

            env->settings->updateSetting("Relaxation.Use", "Dual", false);

            env->settings->updateSetting("Reformulation.Constraint.PartitionNonlinearTerms", "Model", true);
            env->settings->updateSetting("Reformulation.Constraint.PartitionQuadraticTerms", "Model", true);
            env->settings->updateSetting("Reformulation.ObjectiveFunction.PartitionNonlinearTerms", "Model", true);
            env->settings->updateSetting("Reformulation.ObjectiveFunction.PartitionQuadraticTerms", "Model", true);
            env->settings->updateSetting("Reformulation.Quadratics.Strategy", "Model", 0);

            env->settings->updateSetting("FixedInteger.CallStrategy", "Primal", 0);
            env->settings->updateSetting("FixedInteger.CreateInfeasibilityCut", "Primal", true);
            env->settings->updateSetting("FixedInteger.Source", "Primal", 0);
            // env->settings->updateSetting("FixedInteger.Warmstart", "Primal", false);
            env->settings->updateSetting("Linesearch.Use", "Primal", false);
            env->settings->updateSetting("Cplex.MIPEmphasis", "Subsolver", 4);
            env->settings->updateSetting("Cplex.NumericalEmphasis", "Subsolver", 1);
            env->settings->updateSetting("Cplex.Probe", "Subsolver", 3);
            env->settings->updateSetting("Cplex.SolnPoolIntensity", "Subsolver", 4);

            break;

        default:
            break;
        }
    }
}

void SHOTSolver::updateSetting(std::string name, std::string category, std::string value)
{
    env->settings->updateSetting(name, category, value);
}

void SHOTSolver::updateSetting(std::string name, std::string category, int value)
{
    env->settings->updateSetting(name, category, value);
}

void SHOTSolver::updateSetting(std::string name, std::string category, bool value)
{
    env->settings->updateSetting(name, category, value);
}

void SHOTSolver::updateSetting(std::string name, std::string category, double value)
{
    env->settings->updateSetting(name, category, value);
}

double SHOTSolver::getDualBound() { return (env->results->getDualBound()); }

double SHOTSolver::getPrimalBound() { return (env->results->getPrimalBound()); }

double SHOTSolver::getAbsoluteObjectiveGap() { return (env->results->getAbsoluteObjectiveGap()); }

double SHOTSolver::getRelativeObjectiveGap() { return (env->results->getRelativeObjectiveGap()); }

int SHOTSolver::getNumberOfPrimalSolutions() { return (env->results->primalSolutions.size() > 0); }

PrimalSolution SHOTSolver::getPrimalSolution()
{
    if(isProblemSolved && env->results->primalSolutions.size() > 0)
    {
        PrimalSolution primalSol = env->results->primalSolutions.at(0);
        return (primalSol);
    }

    PrimalSolution primalSol;
    return (primalSol);
}

std::vector<PrimalSolution> SHOTSolver::getPrimalSolutions() { return (env->results->primalSolutions); }

E_TerminationReason SHOTSolver::getTerminationReason() { return (env->results->terminationReason); }
} // namespace SHOT