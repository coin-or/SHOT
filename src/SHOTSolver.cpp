#include "SHOTSolver.h"

SHOTSolver::SHOTSolver() :
		gms2os(NULL)
{
	initializeSettings();
	isProblemInitialized = false;
}

SHOTSolver::~SHOTSolver()
{
	if (isProblemInitialized) delete solutionStrategy;
	delete gms2os;
}

bool SHOTSolver::setOptions(std::string fileName)
{
	FileUtil *fileUtil = new FileUtil();
	OSoLReader *osolreader = new OSoLReader();

	try
	{
		std::string fileContents;
		std::string fileExtension = boost::filesystem::extension(fileName);

		if (fileExtension == ".xml" || fileExtension == ".osol")
		{
			fileContents = fileUtil->getFileAsString(fileName.c_str());
			Settings::getInstance().readSettingsFromOSoL(fileContents);
		}
		else if (fileExtension == ".opt")
		{
			fileContents = fileUtil->getFileAsString(fileName.c_str());
			Settings::getInstance().readSettingsFromGAMSOptFormat(fileContents);
		}
		else
		{
			ProcessInfo::getInstance().outputError(
					"Error when reading options from \"" + fileName + "\". File extension must be osol, xml or opt.");
		}

	}
	catch (const ErrorClass& eclass)
	{
		ProcessInfo::getInstance().outputError("Error when reading options from \"" + fileName + "\"", eclass.errormsg);
		delete fileUtil;
		delete osolreader;
		return (false);
	}

	// Sets the correct log levels
	osoutput->SetPrintLevel("stdout",
			(ENUM_OUTPUT_LEVEL)(Settings::getInstance().getIntSetting("LogLevelConsole", "SHOTSolver") + 1));
	osoutput->SetPrintLevel("shotlogfile",
			(ENUM_OUTPUT_LEVEL)(Settings::getInstance().getIntSetting("LogLevelFile", "SHOTSolver") + 1));

	ProcessInfo::getInstance().outputAlways("Options read from file \"" + fileName + "\"");

	delete fileUtil;
	delete osolreader;

	return (true);
}

bool SHOTSolver::setOptions(OSOption *osOptions)
{
	try
	{
		Settings::getInstance().readSettingsFromOSOption(osOptions);
	}
	catch (ErrorClass &eclass)
	{

		ProcessInfo::getInstance().outputError("Error when reading options.", eclass.errormsg);

		return (false);
	}

	ProcessInfo::getInstance().outputInfo("Options read.");

	return (true);
}

bool SHOTSolver::setProblem(std::string fileName)
{
	OSInstance *tmpInstance;

	try
	{
		auto tmpFilename = fileName;

		std::string::size_type idx;
		string file_extension;

		idx = tmpFilename.rfind('.');

		if (idx != std::string::npos)
		{
			file_extension = tmpFilename.substr(idx + 1);
		}
		else
		{
			ProcessInfo::getInstance().outputError("No filetype specified.");

			delete tmpInstance;

			return (false);
		}

		if (file_extension == "osil" || file_extension == "xml")
		{
			FileUtil *fileUtil = new FileUtil();
			std::string fileContents = fileUtil->getFileAsString(tmpFilename.c_str());
			delete fileUtil;

			OSiLReader *osilreader = new OSiLReader();
			tmpInstance = osilreader->readOSiL(fileContents);

			//delete osilreader;
			//TODO: Why can't osilreader be deleted
		}
		else if (file_extension == "nl")
		{
			OSnl2OS *nl2os = new OSnl2OS();
			nl2os->readNl(tmpFilename);
			nl2os->createOSObjects();
			tmpInstance = nl2os->osinstance;
		}
		else if (file_extension == "gms")
		{
			assert(gms2os == NULL);
			gms2os = new GAMS2OS();
			gms2os->readGms(tmpFilename);
			gms2os->createOSObjects();

			tmpInstance = gms2os->osinstance;
		}
		else if (file_extension == "dat")
		{
			assert(gms2os == NULL);
			gms2os = new GAMS2OS();
			gms2os->readCntr(tmpFilename);
			gms2os->createOSObjects();
			tmpInstance = gms2os->osinstance;
		}
		else
		{
			ProcessInfo::getInstance().outputError("Wrong filetype specified.");

			delete tmpInstance;

			return (false);
		}

		tmpInstance->instanceHeader->source = tmpFilename;
	}
	catch (const ErrorClass& eclass)
	{
		ProcessInfo::getInstance().outputError("Error when reading problem from \"" + fileName + "\"", eclass.errormsg);

		delete tmpInstance;

		return (false);
	}

	//Removes path
	std::string tmpFile = fileName.substr(fileName.find_last_of("/\\") + 1);

	//Removes file extension (if any)
	size_t lastdot = tmpFile.find_last_of(".");
	if (lastdot != std::string::npos) tmpFile = tmpFile.substr(0, lastdot);

	tmpInstance->setInstanceName(tmpFile);

	Settings::getInstance().updateSetting("ProblemFile", "SHOTSolver", fileName);
	Settings::getInstance().updateSetting("DebugPath", "SHOTSolver", "problemdebug/" + tmpFile);

	if (Settings::getInstance().getBoolSetting("Debug", "SHOTSolver")) initializeDebugMode();

	bool status = this->setProblem(tmpInstance);

	return (status);
}

bool SHOTSolver::setProblem(OSInstance *osInstance)
{
	solutionStrategy = new SolutionStrategySHOT(osInstance);
	isProblemInitialized = true;

	return (true);
}

bool SHOTSolver::solveProblem()
{
	bool result = solutionStrategy->solveProblem();

	if (result && gms2os != NULL)
	{
		gms2os->writeResult(ProcessInfo::getInstance());
	}

	return (result);
}

std::string SHOTSolver::getOSrL()
{
	return (ProcessInfo::getInstance().getOSrl());
}

std::string SHOTSolver::getOSoL()
{
	if (!Settings::getInstance().settingsInitialized) initializeSettings();

	return (Settings::getInstance().getSettingsInOSolFormat());
}

std::string SHOTSolver::getGAMSOptFile()
{
	if (!Settings::getInstance().settingsInitialized) initializeSettings();

	return (Settings::getInstance().getSettingsInGAMSOptFormat());
}

std::string SHOTSolver::getTraceResult()
{
	return (ProcessInfo::getInstance().getTraceResult());
}

void SHOTSolver::initializeSettings()
{
	if (Settings::getInstance().settingsInitialized)
	{
		ProcessInfo::getInstance().outputWarning(
				"Warning! Settings have already been initialized. Ignoring new settings.");
		return;
	}

	ProcessInfo::getInstance().outputInfo("Starting initialization of settings:");

// Logging setting
	std::vector < std::string > enumLogLevel;
	enumLogLevel.push_back("error");
	enumLogLevel.push_back("summary");
	enumLogLevel.push_back("warning");
	enumLogLevel.push_back("info");
	enumLogLevel.push_back("debug");
	enumLogLevel.push_back("trace");
	enumLogLevel.push_back("detailed_trace");
	Settings::getInstance().createSetting("LogLevelConsole", "SHOTSolver",
			static_cast<int>(ENUM_OUTPUT_LEVEL_summary) - 1, "Log level for console output", enumLogLevel);

	Settings::getInstance().createSetting("LogLevelFile", "SHOTSolver", static_cast<int>(ENUM_OUTPUT_LEVEL_summary) - 1,
			"Log level for file output", enumLogLevel);

	enumLogLevel.clear();

	Settings::getInstance().createSetting("SaveAllPrimalSolutions", "SHOTSolver", false,
			"Should all intermediate solutions (primal and dual) be saved.");

// Solution strategy
	std::vector < std::string > enumSolutionStrategy;
	enumSolutionStrategy.push_back("ESH");
	enumSolutionStrategy.push_back("ECP");
	Settings::getInstance().createSetting("SolutionStrategy", "Algorithm", static_cast<int>(ES_SolutionStrategy::ESH),
			"Solution strategy", enumSolutionStrategy);
	enumSolutionStrategy.clear();

// Relaxation
	Settings::getInstance().createSetting("IterLimitLP", "Algorithm", 200, "LP iteration limit for solver", 0,
			OSINT_MAX);
	Settings::getInstance().createSetting("TimeLimitLP", "Algorithm", 30.0, "LP time limit for solver", 0, OSDBL_MAX);
	Settings::getInstance().createSetting("ObjectiveStagnationIterationLimit", "Algorithm", 100,
			"Max number of iterations without objective function improvement", 0, OSINT_MAX);
	Settings::getInstance().createSetting("ObjectiveStagnationTolerance", "Algorithm", 0.000001,
			"Objective function improvement tolerance", 0.0, OSDBL_MAX);
	Settings::getInstance().createSetting("IterSolveLPRelaxation", "Algorithm", 0,
			"Solve an LP relaxation each number of iterations, 0= disable ", 0, OSINT_MAX);

	Settings::getInstance().createSetting("SolveFixedLP", "Algorithm", true,
			"Solve an LP with fixed integers if integer-values have not changes in several MIP iterations.");

	Settings::getInstance().createSetting("SolveFixedLPMaxIter", "Algorithm", 20,
			"Max LP iterations for fixed strategy", 0, OSINT_MAX);
	Settings::getInstance().createSetting("SolveFixedLPObjTol", "Algorithm", 0.001,
			"Objective tolerance for fixed strategy", 0.0, OSDBL_MAX);
	Settings::getInstance().createSetting("SolveFixedLPConstrTol", "Algorithm", 0.00001,
			"Constraint tolerance for fixed strategy", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("AddIntegerCuts", "Algorithm", true,
			"Add integer cuts for infeasible integer-combinations for binary problems");

	std::vector < std::string > enumRelaxationStrategy;
	enumRelaxationStrategy.push_back("Standard");
	enumRelaxationStrategy.push_back("Adaptive");
	Settings::getInstance().createSetting("RelaxationStrategy", "Algorithm",
			static_cast<int>(ES_RelaxationStrategy::Standard), "Relaxation strategy", enumRelaxationStrategy);
	enumSolutionStrategy.clear();

// Termination tolerance setting
	Settings::getInstance().createSetting("ConstrTermTolMILP", "Algorithm", 1e-8,
			"Final termination tolerance for constraints", 0, OSDBL_MAX);
	Settings::getInstance().createSetting("ConstrTermTolInitialFactor", "Algorithm", 100.0,
			"Factor for constraint tolerance at initial MILP tree limit ", 1.0, OSDBL_MAX);
	Settings::getInstance().createSetting("ConstrTermTolLP", "Algorithm", 0.5,
			"Termination tolerance in constraints for LP preprocessing");
	Settings::getInstance().createSetting("ObjectionFunctionTol", "Algorithm", 1e-5,
			"Additional termination tolerance for the linear and nonlinear objective function.");
	Settings::getInstance().createSetting("GapTermTolAbsolute", "Algorithm", 0.001,
			"Absolute gap tolerance for objective function", 0, OSDBL_MAX);
	Settings::getInstance().createSetting("GapTermTolRelative", "Algorithm", 0.001,
			"Relative gap tolerance for objective function", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("TimeLimit", "Algorithm", 900.0, "Time limit in seconds for solver", 0.0,
			OSDBL_MAX);
	Settings::getInstance().createSetting("IterLimitMILP", "Algorithm", 2000, "MILP iteration limit for solver", 1,
			OSINT_MAX);

// Max Solution pool size
	Settings::getInstance().createSetting("SolutionPoolSize", "MILP", 100, "Solution pool capacity", 0, OSINT_MAX);

	Settings::getInstance().createSetting("MaxHyperplanesPerIteration", "Algorithm", 200,
			"Maximal extra number of hyperplanes to add per iteration", 0, OSINT_MAX);

// Interiorpoint solver
	std::vector < std::string > enumNLPSolver;
	enumNLPSolver.push_back("CuttingPlaneMinimax");
	enumNLPSolver.push_back("IpoptMinimax");
	enumNLPSolver.push_back("IpoptRelaxed");
	enumNLPSolver.push_back("IpoptMinimax and IpoptRelaxed");

	Settings::getInstance().createSetting("InteriorPointSolver", "InteriorPoint",
			static_cast<int>(ES_NLPSolver::CuttingPlaneMiniMax), "NLP solver", enumNLPSolver);
	enumNLPSolver.clear();

// NLP empty bound
	Settings::getInstance().createSetting("MinimaxObjectiveBound", "InteriorPoint", 10000000000.0,
			"Value for obj. function in NLP minimax problem", 0, OSDBL_MAX);

	std::vector < std::string > enumIPOptSolver;
	enumIPOptSolver.push_back("ma27");
	enumIPOptSolver.push_back("ma57");
	enumIPOptSolver.push_back("ma86");
	enumIPOptSolver.push_back("ma97");
	enumIPOptSolver.push_back("mumps");
	Settings::getInstance().createSetting("IpoptSolver", "Ipopt", static_cast<int>(ES_IPOptSolver::ma57),
			"Linear subsolver in Ipopt", enumIPOptSolver);
	enumIPOptSolver.clear();

	Settings::getInstance().createSetting("ToleranceInteriorPointStrategy", "Ipopt", 1E-8,
			"Relative convergence tolerance in interior point search strategy");
	Settings::getInstance().createSetting("MaxIterInteriorPointStrategy", "Ipopt", 1000,
			"Maximum number of iterations in interior point search strategy");
	Settings::getInstance().createSetting("ConstraintToleranceInteriorPointStrategy", "Ipopt", 1E-8,
			"Constraint violation tolerance", -OSDBL_MAX, OSDBL_MAX);

//SHOT cutting plane Minimax NLP solver
	Settings::getInstance().createSetting("IterLimit", "InteriorPointCuttingPlane", 200,
			"LP iteration limit for solver", 0, OSINT_MAX);
	Settings::getInstance().createSetting("IterLimitSubsolver", "InteriorPointCuttingPlane", 1000,
			"Iteration limit for minimization subsolver", 0, OSINT_MAX);
	Settings::getInstance().createSetting("BitPrecision", "InteriorPointCuttingPlane", 8,
			"Required bit precision for minimization subsolver", 1, 64);
	Settings::getInstance().createSetting("TermToleranceAbs", "InteriorPointCuttingPlane", 0.5,
			"Absolute termination tolerance for the difference between LP and linesearch objective", 0.0, OSDBL_MAX);
	Settings::getInstance().createSetting("TermToleranceRel", "InteriorPointCuttingPlane", 0.1,
			"Relative termination tolerance for the difference between LP and linesearch objective", 0.0, OSDBL_MAX);
	Settings::getInstance().createSetting("ConstraintSelectionTolerance", "InteriorPointCuttingPlane", 0.05,
			"The tolerance for selecting the most constraint with largest deviation", 0.0, 1.0);

	Settings::getInstance().createSetting("CopyCuttingPlanes", "InteriorPointCuttingPlane", true,
			"Copy over the valid cutting planes in the minimax solver to main problem.");

	Settings::getInstance().createSetting("MinimaxUpperBound", "InteriorPoint", 0.1,
			"Upper bound for minimax objective variable", -OSDBL_MAX, OSDBL_MAX);
	Settings::getInstance().createSetting("OriginalObjectiveWeight", "InteriorPointRelaxed", 0.0,
			"The weight of the original objective function in NLP relaxation", -OSDBL_MAX, OSDBL_MAX);

// MILP/LP solver
	std::vector < std::string > enumMILPSolver;
	enumMILPSolver.push_back("Cplex");
	enumMILPSolver.push_back("Gurobi");
	enumMILPSolver.push_back("Cbc");
	enumMILPSolver.push_back("CplexExperimental");
	Settings::getInstance().createSetting("MILPSolver", "MILP", static_cast<int>(ES_MILPSolver::Cplex), "MILP solver",
			enumMILPSolver);
	enumMILPSolver.clear();

	Settings::getInstance().createSetting("PopulateSolutionPool", "MILP", false,
			"Try to fill the solution pool after solver finishes");

// Initial MILP solution depth
	Settings::getInstance().createSetting("MILPSolLimitInitial", "MILP", 1, "Initial MILP solution limit", 1,
			OSINT_MAX);
	Settings::getInstance().createSetting("MILPSolLimitEnd", "MILP", 10.0,
			"End MILP solution limit when final constraint tolerance is reached ", 1.0, OSDBL_MAX);
	Settings::getInstance().createSetting("MILPSolIncreaseIter", "MILP", 50,
			"Max number of iterations between MILP solution limit increase", 0, OSINT_MAX);
	Settings::getInstance().createSetting("ForceOptimalIter", "MILP", 10000,
			"Number of iterations without dual bound update to force optimal MILP solution", 0, OSINT_MAX);
	Settings::getInstance().createSetting("ForceOptimalTime", "MILP", 1000.0,
			"Amount of time without dual bound update to force optimal MILP solution", 0, OSDBL_MAX);
	Settings::getInstance().createSetting("MILPSolLimitUpdateTol", "MILP", 0.001,
			"The constraint tolerance to update solution limit at", 0, OSDBL_MAX);

// Presolve
	std::vector < std::string > enumPresolve;
	enumPresolve.push_back("Never");
	enumPresolve.push_back("Once");
	enumPresolve.push_back("Always");
	Settings::getInstance().createSetting("PresolveStrategy", "Presolve", static_cast<int>(ES_PresolveStrategy::Once),
			"What presolve strategy to use", enumPresolve);
	enumPresolve.clear();

	Settings::getInstance().createSetting("UsePresolveBoundsForPrimalNLP", "Presolve", false,
			"Warning! Does not seem to work. Use updated bounds from the MIP solver when solving primal NLP problems");
	Settings::getInstance().createSetting("UsePresolveBoundsForMIP", "Presolve", true,
			"Use updated bounds from the MIP solver in new MIP iterations");
	Settings::getInstance().createSetting("RemoveRedundantConstraintsFromMIP", "Presolve", false,
			"Removes the constraints flagged as redundant by presolve");

// Add constraints as lazy constraints
	Settings::getInstance().createSetting("UseLazyConstraints", "MILP", false,
			"Add supporting hyperplanes as lazy constraints");
	Settings::getInstance().createSetting("UsePrimalObjectiveCut", "MILP", true,
			"Add an objective cut in the primal solution");
	Settings::getInstance().createSetting("UpdateNonlinearObjectiveVariableBounds", "MILP", false,
			"Updates the bounds for the nonlinear objective variable when new dual/primal bounds are found ");

	Settings::getInstance().createSetting("DelayedConstraints", "MILP", true,
			"Add supporting hyperplanes only after optimal MILP solution (=1).");

//Settings::getInstance().createSetting("UseQuadraticProgramming", "Algorithm", true, "Solve QP problems if objective function is quadratic,");

	std::vector < std::string > enumQPStrategy;
	enumQPStrategy.push_back("All nonlinear");
	enumQPStrategy.push_back("Use quadratic objective");
	enumQPStrategy.push_back("Use quadratic constraints");
	Settings::getInstance().createSetting("QPStrategy", "Algorithm",
			static_cast<int>(ES_QPStrategy::QuadraticObjective), "QP strategy", enumQPStrategy);

// CPLEX
	Settings::getInstance().createSetting("SolnPoolIntensity", "CPLEX", 0,
			"0 = automatic, 1 = mild, 2 = moderate, 3 = aggressive, 4 = very aggressive", 0, 4);
	Settings::getInstance().createSetting("SolnPoolReplace", "CPLEX", 2,
			"0 = replace oldest solution, 1 = replace worst solution, 2 = find diverse solutions", 0, 2);

	Settings::getInstance().createSetting("SolnPoolGap", "CPLEX", 1.0e+75,
			"The relative tolerance on the objective values in the solution pool", 0, 1.0e+75);
	Settings::getInstance().createSetting("MIPEmphasis", "CPLEX", 1,
			"0 = balanced, 1 = feasibility, 2 = optimality, 3 = best bound, 4 = hidden feasible", 0, 4);

	Settings::getInstance().createSetting("Probe", "CPLEX", 0,
			"-1 = no probing, 0 = automatic, 1 = moderate, 2 = aggressive, 3 = very aggressive", -1, 3);

	Settings::getInstance().createSetting("ParallelMode", "CPLEX", 0,
			"-1 = opportunistic, 0 = automatic, 1 = deterministic", -1, 1);

	Settings::getInstance().createSetting("Threads", "CPLEX", 0, "Number of threads to use, 0 = automatic", 0, 999);

// Linesearch
	std::vector < std::string > enumLinesearchMethod;
	enumLinesearchMethod.push_back("BoostTOMS748");
	enumLinesearchMethod.push_back("BoostBisection");
	enumLinesearchMethod.push_back("Bisection");
	Settings::getInstance().createSetting("LinesearchMethod", "Linesearch",
			static_cast<int>(ES_LinesearchMethod::BoostTOMS748), "Linesearch method", enumLinesearchMethod);
	enumLinesearchMethod.clear();

	Settings::getInstance().createSetting("LinesearchLambdaEps", "Linesearch", 1e-16,
			"Epsilon lambda tolerance for linesearch", 0.0, OSDBL_MAX);
	Settings::getInstance().createSetting("LinesearchMaxIter", "Linesearch", 100, "Maximal iterations for linesearch",
			0, OSINT_MAX);
	Settings::getInstance().createSetting("LinesearchConstrEps", "Linesearch", 0.0,
			"Epsilon constraint tolerance for linesearch", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("LinesearchConstraintSelectionFactor", "ECP", 0.0,
			"The fraction of violated constraints to generate supporting hyperplanes for when using the ECP strategy.",
			0.0, 1.0);

	Settings::getInstance().createSetting("LinesearchConstraintStrategy", "ESH", 0.0,
			"NOT USED! The fraction of violated constraints to generate supporting hyperplanes for when using the ECP strategy.",
			0.0, 1.0);

	std::vector < std::string > enumLinesearchConstraintStrategy;
	enumLinesearchConstraintStrategy.push_back("Max function");
	enumLinesearchConstraintStrategy.push_back("Individual constraints");
	Settings::getInstance().createSetting("LinesearchConstraintStrategy", "ESH",
			static_cast<int>(ES_LinesearchConstraintStrategy::AllAsMaxFunct),
			"Strategy for grouping the constraints for linesearches", enumLinesearchConstraintStrategy);
	enumLinesearchMethod.clear();

	Settings::getInstance().createSetting("LinesearchConstraintTolerance", "ESH", 1e-8,
			"Constraint tolerance for when not to add individual hyperplanes.", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("LinesearchConstraintFactor", "ESH", 0.5,
			"No linesearch on a constraint if its solution point value is less than this factor of the maximum.", 1e-6,
			1.0);

	std::string empty = "empty";

	Settings::getInstance().createSetting("ProblemFile", "SHOTSolver", empty, "The filename of the problem.", true);
	Settings::getInstance().createSetting("DebugPath", "SHOTSolver", empty,
			"The path where to save the debug information", true);
	Settings::getInstance().createSetting("Debug", "SHOTSolver", false, "Use debug functionality");

// Primal bound
	std::vector < std::string > enumPrimalNLPSolver;
	enumPrimalNLPSolver.push_back("CuttingPlane");
	enumPrimalNLPSolver.push_back("Ipopt");
	enumPrimalNLPSolver.push_back("GAMS");

	Settings::getInstance().createSetting("PrimalNLPSolver", "PrimalBound", static_cast<int>(ES_PrimalNLPSolver::IPOpt),
			"NLP solver from fixed NLP primal bound strategy.", enumPrimalNLPSolver);
	enumPrimalNLPSolver.clear();

	std::vector < std::string > enumPrimalNLPStrategy;
	enumPrimalNLPStrategy.push_back("Use each iteration");
	enumPrimalNLPStrategy.push_back("Don't use");
	enumPrimalNLPStrategy.push_back("Based on iteration or time");
	enumPrimalNLPStrategy.push_back("Based on iteration or time, and for all feasible MIP solutions");

	Settings::getInstance().createSetting("NLPFixedStrategy", "PrimalBound",
			static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions),
			"How to use fixed NLP primal calls", enumPrimalNLPStrategy);
	enumPrimalNLPStrategy.clear();

	Settings::getInstance().createSetting("NLPFixedWarmstart", "PrimalBound", true,
			"Use a warm start for the NLP solver");

	Settings::getInstance().createSetting("NLPFixedMaxIters", "PrimalBound", 10,
			"Maximal number of iterations between calls", 0, OSINT_MAX);
	Settings::getInstance().createSetting("NLPFixedMaxElapsedTime", "PrimalBound", 5.0,
			"Maximal elapsed time between calls", 0, OSDBL_MAX);
	Settings::getInstance().createSetting("NLPTimeLimit", "PrimalBound", 10.0,
			"Maximal time allowed per fixed NLP problem", 0, OSDBL_MAX);
	Settings::getInstance().createSetting("NLPIterLimit", "PrimalBound", 10000000,
			"Maximum number of iterations allowed per fixed NLP problem", 0, OSINT_MAX);

	std::string solver = "conopt";
	std::string optfile = "conopt.opt";

	Settings::getInstance().createSetting("NLPSolver", "GAMS", solver, "Name of the NLP solver to use in GAMS");
	Settings::getInstance().createSetting("NLPOptionsFile", "GAMS", optfile,
			"Name of the options file for the NLP solver in GAMS");
	Settings::getInstance().createSetting("ShowOutput", "GAMS", false, "Show GAMS output");

	std::vector < std::string > enumAddPrimalPointAsInteriorPoint;
	enumAddPrimalPointAsInteriorPoint.push_back("Keep original");
	enumAddPrimalPointAsInteriorPoint.push_back("Keep both");
	enumAddPrimalPointAsInteriorPoint.push_back("Only new");
	enumAddPrimalPointAsInteriorPoint.push_back("Only average");
	Settings::getInstance().createSetting("AddPrimalBoundAsInteriorPoint", "Algorithm",
			static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepOriginal),
			"How to update interior point when new primal solution is found", enumAddPrimalPointAsInteriorPoint);
	enumAddPrimalPointAsInteriorPoint.clear();

	Settings::getInstance().createSetting("UseObjectiveLinesearch", "PrimalBound", true,
			"Use a linesearch to find a primal bound if objective nonlinear");
	std::vector < std::string > enumPrimalBoundNLPStartingPoint;
	enumPrimalBoundNLPStartingPoint.push_back("All solutions");
	enumPrimalBoundNLPStartingPoint.push_back("First solution");
	enumPrimalBoundNLPStartingPoint.push_back("All feasible solutions");
	enumPrimalBoundNLPStartingPoint.push_back("First and all feasible solutions");
	enumPrimalBoundNLPStartingPoint.push_back("Smallest constraint deviation solution");
	Settings::getInstance().createSetting("NLPFixedSource", "PrimalBound",
			static_cast<int>(ES_PrimalBoundNLPFixedPoint::FirstAndFeasibleSolutions), "Fixed MIP point source",
			enumPrimalBoundNLPStartingPoint);
	enumPrimalBoundNLPStartingPoint.clear();

	Settings::getInstance().createSetting("PrimalBoundNonlinearTolerance", "PrimalBound", 1e-7,
			"The nonlinear constraint tolerance for accepting primal bounds ");

	Settings::getInstance().createSetting("PrimalBoundLinearTolerance", "PrimalBound", 1e-6,
			"The linear constraint tolerance for accepting primal bounds ");

	Settings::getInstance().createSetting("PrimalBoundIntegerTolerance", "PrimalBound", 1e-6,
			"The tolerance for accepting primal bounds ");

	ProcessInfo::getInstance().outputInfo("Initialization of settings complete.");

	Settings::getInstance().settingsInitialized = true;
}

void SHOTSolver::initializeDebugMode()
{
	auto debugPath = Settings::getInstance().getStringSetting("DebugPath", "SHOTSolver");

	boost::filesystem::path debugDir(boost::filesystem::current_path() / debugPath);

	if (boost::filesystem::exists(debugDir))
	{
		ProcessInfo::getInstance().outputInfo("Debug directory " + debugPath + " already exists.");
	}
	else
	{
		if (boost::filesystem::create_directories(debugDir))
		{
			ProcessInfo::getInstance().outputInfo("Debug directory " + debugPath + " created.");
		}
		else
		{
			ProcessInfo::getInstance().outputWarning("Could not create debug directory.");
		}
	}

	boost::filesystem::path source(Settings::getInstance().getStringSetting("ProblemFile", "SHOTSolver"));
	boost::filesystem::copy_file(boost::filesystem::canonical(source), debugDir / source.filename(),
			boost::filesystem::copy_option::overwrite_if_exists);

	FileUtil* fileUtil = new FileUtil();

	fileUtil->writeFileFromString(debugPath + "/options.xml", getOSoL());

	delete fileUtil;
}
