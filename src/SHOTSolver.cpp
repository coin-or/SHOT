#include "SHOTSolver.h"

SHOTSolver::SHOTSolver()
{
	settings = SHOTSettings::Settings::getInstance();
	processInfo = ProcessInfo::getInstance();

	initializeSettings();
}

SHOTSolver::~SHOTSolver()
{
	delete settings;
	delete processInfo;
	delete solutionStrategy;
}

bool SHOTSolver::setOptions(std::string fileName)
{
	FileUtil *fileUtil = new FileUtil();
	OSoLReader *osolreader = new OSoLReader();

	try
	{
		std::string osol = fileUtil->getFileAsString(fileName.c_str());
		settings->readSettings(osol);
	}
	catch (const ErrorClass& eclass)
	{
		processInfo->outputError("Error when reading options from \"" + fileName + "\"", eclass.errormsg);
		delete fileUtil;
		delete osolreader;
		return (false);
	}

	// Sets the correct log levels
	osoutput->SetPrintLevel("stdout",
			(ENUM_OUTPUT_LEVEL)(settings->getIntSetting("LogLevelConsole", "SHOTSolver") + 1));
	osoutput->SetPrintLevel("shotlogfile",
			(ENUM_OUTPUT_LEVEL)(settings->getIntSetting("LogLevelFile", "SHOTSolver") + 1));

	processInfo->outputInfo("Options read from file \"" + fileName + "\"");

	delete fileUtil;
	delete osolreader;

	return (true);
}

bool SHOTSolver::setOptions(OSOption *osOptions)
{
	try
	{
		settings->readSettings(osOptions);
	}
	catch (ErrorClass &eclass)
	{

		processInfo->outputError("Error when reading options.", eclass.errormsg);

		return (false);
	}

	processInfo->outputInfo("Options read.");

	return (true);
}

bool SHOTSolver::setProblem(std::string fileName)
{
	FileUtil *fileUtil = new FileUtil();
	OSiLReader *osilreader = new OSiLReader();
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
			// No extension found
		}

		if (file_extension == "osil")
		{
			std::string fileContents = fileUtil->getFileAsString(tmpFilename.c_str());
			tmpInstance = osilreader->readOSiL(fileContents);

		}
		else if (file_extension == "nl")
		{
			OSnl2OS *nl2os = new OSnl2OS();
			nl2os->readNl(tmpFilename);
			nl2os->createOSObjects();
			tmpInstance = nl2os->osinstance;
		}
		else
		{
			processInfo->outputError("Wrong filetype specified.");
			return (false);
		}

		tmpInstance->instanceHeader->source = tmpFilename;
	}
	catch (const ErrorClass& eclass)
	{
		delete fileUtil;
		delete osilreader;

		processInfo->outputError("Error when reading problem from \"" + fileName + "\"", eclass.errormsg);

		return (false);
	}

	//Removes path
	std::string tmpFile = fileName.substr(fileName.find_last_of("/\\") + 1);

	//Removes file extension (if any)
	size_t lastdot = tmpFile.find_last_of(".");
	if (lastdot != std::string::npos) tmpFile = tmpFile.substr(0, lastdot);

	tmpInstance->setInstanceName(tmpFile);

	settings->updateSetting("ProblemFile", "SHOTSolver", fileName);
	settings->updateSetting("DebugPath", "SHOTSolver", "problemdebug/" + tmpFile);

	if (settings->getBoolSetting("Debug", "SHOTSolver")) initializeDebugMode();
	else
	{
		//getOSol();
		initializeDebugMode(); // Does not work without this...
		//std::cout << getOSol(); // Needed due to unknown reason

		//FileUtil* fileUtil = new FileUtil();
		/*auto debugPath = settings->getStringSetting("DebugPath", "SHOTSolver");

		 boost::filesystem::path debugDir(boost::filesystem::current_path() / debugPath);

		 if (boost::filesystem::exists(debugDir))
		 {
		 processInfo->logger.message(1) << "Debug directory " << debugPath << " already exists." << CoinMessageEol;
		 }
		 else
		 {
		 if (boost::filesystem::create_directories(debugDir))
		 {
		 processInfo->logger.message(1) << "Debug directory " << debugPath << " created." << CoinMessageEol;
		 }
		 else
		 {
		 processInfo->logger.message(1) << "Could not create debug directory " << debugPath << "!"
		 << CoinMessageEol;
		 }
		 }

		 boost::filesystem::path source(settings->getStringSetting("ProblemFile", "SHOTSolver"));
		 boost::filesystem::copy_file(boost::filesystem::canonical(source), debugDir / source.filename(),
		 boost::filesystem::copy_option::overwrite_if_exists);

		 //fileUtil->writeFileFromString(debugPath + "/options.xml", getOSol());
		 */
		//delete fileUtil;
		//std::cout << "HEJ" << std::endl;
	}

	bool status = this->setProblem(tmpInstance);

	delete fileUtil;
	fileUtil = NULL;

	return (status);
}

bool SHOTSolver::setProblem(OSInstance *osInstance)
{
	solutionStrategy = new SolutionStrategySHOT(osInstance);

	return (true);
}

bool SHOTSolver::solveProblem()
{
	return (solutionStrategy->solveProblem());
}

std::string SHOTSolver::getOSrl()
{
	return (processInfo->getOSrl());
}

std::string SHOTSolver::getOSol()
{
	if (!settings->settingsInitialized) initializeSettings();

	return (settings->getSettingsAsOSol());
}

std::string SHOTSolver::getTraceResult()
{
	return (processInfo->getTraceResult());
}

void SHOTSolver::initializeSettings()
{
	if (settings->settingsInitialized)
	{
		processInfo->outputWarning("Warning! Settings have already been initialized. Ignoring new settings.");
		return;
	}

	processInfo->outputInfo("Starting initialization of settings:");

	// Logging setting
	std::vector < std::string > enumLogLevel;
	enumLogLevel.push_back("error");
	enumLogLevel.push_back("summary");
	enumLogLevel.push_back("warning");
	enumLogLevel.push_back("info");
	enumLogLevel.push_back("debug");
	enumLogLevel.push_back("trace");
	enumLogLevel.push_back("detailed_trace");
	settings->createSetting("LogLevelConsole", "SHOTSolver", static_cast<int>(ENUM_OUTPUT_LEVEL_summary) - 1,
			"Log level for console output", enumLogLevel);

	settings->createSetting("LogLevelFile", "SHOTSolver", static_cast<int>(ENUM_OUTPUT_LEVEL_summary) - 1,
			"Log level for file output", enumLogLevel);

	enumLogLevel.clear();

	// Solution strategy
	std::vector < std::string > enumSolutionStrategy;
	enumSolutionStrategy.push_back("ESH");
	enumSolutionStrategy.push_back("ECP");
	settings->createSetting("SolutionStrategy", "Algorithm", static_cast<int>(ES_SolutionStrategy::ESH),
			"Solution strategy", enumSolutionStrategy);
	enumSolutionStrategy.clear();

	// Relaxation
	settings->createSetting("IterLimitLP", "Algorithm", 200, "LP iteration limit for solver", 0, OSINT_MAX);
	settings->createSetting("ObjectiveStagnationIterationLimit", "Algorithm", 100,
			"Max number of iterations without objective function improvement", 0, OSINT_MAX);
	settings->createSetting("ObjectiveStagnationTolerance", "Algorithm", 0.000001,
			"Objective function improvement tolerance", 0.0, OSDBL_MAX);
	settings->createSetting("IterSolveLPRelaxation", "Algorithm", 0,
			"Solve an LP relaxation each number of iterations, 0= disable ", 0, OSINT_MAX);

	settings->createSetting("SolveFixedLP", "Algorithm", true,
			"Solve an LP with fixed integers if integer-values have not changes in several MIP iterations.");

	settings->createSetting("SolveFixedLPMaxIter", "Algorithm", 5, "Max LP iterations for fixed strategy", 0,
			OSINT_MAX);
	settings->createSetting("SolveFixedLPObjTol", "Algorithm", 0.001, "Objective tolerance for fixed strategy", 0.0,
			OSDBL_MAX);
	settings->createSetting("SolveFixedLPConstrTol", "Algorithm", 0.00001, "Constraint tolerance for fixed strategy",
			0.0, OSDBL_MAX);

	std::vector < std::string > enumRelaxationStrategy;
	enumRelaxationStrategy.push_back("Standard");
	enumRelaxationStrategy.push_back("Adaptive");
	settings->createSetting("RelaxationStrategy", "Algorithm", static_cast<int>(ES_RelaxationStrategy::Standard),
			"Relaxation strategy", enumRelaxationStrategy);
	enumSolutionStrategy.clear();

	// Termination tolerance setting
	settings->createSetting("ConstrTermTolMILP", "Algorithm", 1e-8, "Final termination tolerance for constraints", 0,
			OSDBL_MAX);
	settings->createSetting("ConstrTermTolInitialFactor", "Algorithm", 100.0,
			"Factor for constraint tolerance at initial MILP tree limit ", 1.0, OSDBL_MAX);
	settings->createSetting("ConstrTermTolLP", "Algorithm", 0.5,
			"Termination tolerance in constraints for LP preprocessing");
	settings->createSetting("ObjectionFunctionTol", "Algorithm", 1e-5,
			"Additional termination tolerance for the linear and nonlinear objective function.");
	settings->createSetting("GapTermTolAbsolute", "Algorithm", 0.001, "Absolute gap tolerance for objective function",
			0, OSDBL_MAX);
	settings->createSetting("GapTermTolRelative", "Algorithm", 0.001, "Relative gap tolerance for objective function",
			0, OSDBL_MAX);

	settings->createSetting("TimeLimit", "Algorithm", 900.0, "Time limit in seconds for solver", 0.0, OSDBL_MAX);
	settings->createSetting("IterLimitMILP", "Algorithm", 2000, "MILP iteration limit for solver", 1, OSINT_MAX);

	// Max Solution pool size
	settings->createSetting("SolutionPoolSize", "MILP", 100, "Solution pool capacity", 0, OSINT_MAX);

	settings->createSetting("MaxHyperplanesPerIteration", "Algorithm", 200,
			"Maximal extra number of hyperplanes to add per iteration", 0, OSINT_MAX);

	// NLP solver
	std::vector < std::string > enumNLPSolver;
	enumNLPSolver.push_back("CuttingPlaneMinimax");
	enumNLPSolver.push_back("IPOptMinimax");
	enumNLPSolver.push_back("IPOptRelaxed");
	enumNLPSolver.push_back("IPOptMinimax and IPOptRelaxed");
	enumNLPSolver.push_back("CouenneMiniMax");
	settings->createSetting("NLPSolver", "NLP", static_cast<int>(ES_NLPSolver::IPOptMiniMax), "NLP solver",
			enumNLPSolver);
	enumNLPSolver.clear();

	// NLP empty bound
	settings->createSetting("NLPObjectiveBound", "NLP", 10000000000.0, "Value for obj. function in NLP problem", 0,
			OSDBL_MAX);

	std::vector < std::string > enumIPOptSolver;
	enumIPOptSolver.push_back("ma27");
	enumIPOptSolver.push_back("ma57");
	enumIPOptSolver.push_back("ma86");
	enumIPOptSolver.push_back("ma97");
	enumIPOptSolver.push_back("mumps");
	enumIPOptSolver.push_back("multiple");
	settings->createSetting("IPOptSolver", "NLP", static_cast<int>(ES_IPOptSolver::ma57), "Linear subsolver in IPOpt",
			enumIPOptSolver);
	enumIPOptSolver.clear();

	//SHOT cutting plane Minimax NLP solver
	settings->createSetting("IterLimit", "MinimaxNLP", 200, "LP iteration limit for solver", 0, OSINT_MAX);
	settings->createSetting("IterLimitSubsolver", "MinimaxNLP", 1000, "Iteration limit for minimization subsolver", 0,
			OSINT_MAX);
	settings->createSetting("BitPrecision", "MinimaxNLP", 8, "Required bit precision for minimization subsolver", 1,
			64);
	settings->createSetting("TermToleranceAbs", "MinimaxNLP", 0.1,
			"Absolute termination tolerance for the difference between LP and linesearch objective", 0.0, OSDBL_MAX);
	settings->createSetting("TermToleranceRel", "MinimaxNLP", 0.001,
			"Relative termination tolerance for the difference between LP and linesearch objective", 0.0, OSDBL_MAX);
	settings->createSetting("ConstraintSelectionTolerance", "MinimaxNLP", 0.05,
			"The tolerance for selecting the most constraint with largest deviation", 0.0, 1.0);
	// Interior point feasibility epsilon
	settings->createSetting("InteriorPointFeasEps", "NLP", 0.000001, "Interior point feasibility epsilon", 0.0,
			OSDBL_MAX);
	settings->createSetting("OriginalObjectiveWeight", "NLP", 0.0,
			"The weight of the original objective function in NLP relaxation", -OSDBL_MAX, OSDBL_MAX);

	// MILP/LP solver
	std::vector < std::string > enumMILPSolver;
	enumMILPSolver.push_back("Cplex");
	enumMILPSolver.push_back("Gurobi");
	enumMILPSolver.push_back("Cbc");
	enumMILPSolver.push_back("CplexExperimental");
	settings->createSetting("MILPSolver", "MILP", static_cast<int>(ES_MILPSolver::Cplex), "MILP solver",
			enumMILPSolver);
	enumMILPSolver.clear();

	settings->createSetting("PopulateSolutionPool", "MILP", false,
			"Try to fill the solution pool after solver finishes");

	// Initial MILP solution depth
	settings->createSetting("MILPSolLimitInitial", "MILP", 1, "Initial MILP solution limit", 1, OSINT_MAX);
	settings->createSetting("MILPSolLimitEnd", "MILP", 10.0,
			"End MILP solution limit when final constraint tolerance is reached ", 1.0, OSDBL_MAX);
	settings->createSetting("MILPSolIncreaseIter", "MILP", 50,
			"Max number of iterations between MILP solution limit increase", 0, OSINT_MAX);
	settings->createSetting("MILPSolForceOptimalIter", "MILP", 10000,
			"Number of iterations without dual bound update to force optimal MILP solution", 0, OSINT_MAX);
	settings->createSetting("MILPSolLimitUpdateTol", "MILP", 0.001,
			"The constraint tolerance to update solution limit at", 0, OSDBL_MAX);

	// Add constraints as lazy constraints
	settings->createSetting("UseLazyConstraints", "MILP", false, "Add supporting hyperplanes as lazy constraints");
	settings->createSetting("UsePrimalObjectiveCut", "MILP", true, "Add an objective cut in the primal solution");

	settings->createSetting("DelayedConstraints", "MILP", true,
			"Add supporting hyperplanes only after optimal MILP solution (=1).");

	//settings->createSetting("UseQuadraticProgramming", "Algorithm", true, "Solve QP problems if objective function is quadratic,");

	std::vector < std::string > enumQPStrategy;
	enumQPStrategy.push_back("All nonlinear");
	enumQPStrategy.push_back("Use quadratic objective");
	enumQPStrategy.push_back("Use quadratic constraints");
	settings->createSetting("QPStrategy", "Algorithm", static_cast<int>(ES_QPStrategy::QuadraticObjective),
			"QP strategy", enumQPStrategy);

	// CPLEX
	settings->createSetting("SolnPoolIntensity", "CPLEX", 0,
			"0 = automatic, 1 = mild, 2 = moderate, 3 = aggressive, 4 = very aggressive", 0, 4);
	settings->createSetting("SolnPoolReplace", "CPLEX", 2,
			"0 = replace oldest solution, 1 = replace worst solution, 2 = find diverse solutions", 0, 2);

	settings->createSetting("SolnPoolGap", "CPLEX", 1.0e+75,
			"The relative tolerance on the objective values in the solution pool", 0, 1.0e+75);
	settings->createSetting("MIPEmphasis", "CPLEX", 1,
			"0 = balanced, 1 = feasibility, 2 = optimality, 3 = best bound, 4 = hidden feasible", 0, 4);

	settings->createSetting("Probe", "CPLEX", 0,
			"-1 = no probing, 0 = automatic, 1 = moderate, 2 = aggressive, 3 = very aggressive", -1, 3);
	// Linesearch
	std::vector < std::string > enumLinesearchMethod;
	enumLinesearchMethod.push_back("Boost");
	enumLinesearchMethod.push_back("Bisection");
	settings->createSetting("LinesearchMethod", "Linesearch", static_cast<int>(ES_LinesearchMethod::Boost),
			"Linesearch method", enumLinesearchMethod);
	enumLinesearchMethod.clear();

	settings->createSetting("LinesearchLambdaEps", "Linesearch", 1e-16, "Epsilon lambda tolerance for linesearch", 0.0,
			OSDBL_MAX);
	settings->createSetting("LinesearchMaxIter", "Linesearch", 100, "Maximal iterations for linesearch", 0, OSINT_MAX);
	settings->createSetting("LinesearchConstrEps", "Linesearch", 0.0, "Epsilon constraint tolerance for linesearch",
			0.0, OSDBL_MAX);

	settings->createSetting("LinesearchConstraintSelectionFactor", "ECP", 0.0,
			"NOT USED! The fraction of violated constraints to generate supporting hyperplanes for when using the ECP strategy.",
			0.0, 1.0);

	settings->createSetting("LinesearchConstraintStrategy", "ESH", 0.0,
			"NOT USED! The fraction of violated constraints to generate supporting hyperplanes for when using the ECP strategy.",
			0.0, 1.0);

	std::vector < std::string > enumLinesearchConstraintStrategy;
	enumLinesearchConstraintStrategy.push_back("Max function");
	enumLinesearchConstraintStrategy.push_back("Individual constraints");
	settings->createSetting("LinesearchConstraintStrategy", "ESH",
			static_cast<int>(ES_LinesearchConstraintStrategy::AllAsMaxFunct),
			"Strategy for grouping the constraints for linesearches", enumLinesearchConstraintStrategy);
	enumLinesearchMethod.clear();

	settings->createSetting("LinesearchConstraintTolerance", "ESH", 1e-8,
			"Constraint tolerance for when not to add individual hyperplanes.", 0, OSDBL_MAX);

	settings->createSetting("LinesearchConstraintFactor", "ESH", 0.5,
			"No linesearch on a constraint if its solution point value is less than this factor of the maximum.", 1e-6,
			1.0);

	// Tracefile
	//createSetting("TraceFile", "Algorithm", "esh.trc", "The filename for the trace file. If empty trace information will not be saved.");

	std::string empty = "empty";

	settings->createSetting("ProblemFile", "SHOTSolver", empty, "The filename of the problem.", true);
	settings->createSetting("DebugPath", "SHOTSolver", empty, "The path where to save the debug information", true);
	settings->createSetting("Debug", "SHOTSolver", false, "Use debug functionality");

	// Primal bound
	std::vector < std::string > enumPrimalNLPStrategy;
	enumPrimalNLPStrategy.push_back("Use each iteration");
	enumPrimalNLPStrategy.push_back("Don't use");
	enumPrimalNLPStrategy.push_back("Based on iteration or time");
	enumPrimalNLPStrategy.push_back("Based on iteration or time, and for all feasible MIP solutions");

	settings->createSetting("NLPFixedStrategy", "PrimalBound",
			static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions),
			"How to use fixed NLP primal calls", enumPrimalNLPStrategy);
	enumPrimalNLPStrategy.clear();

	settings->createSetting("NLPFixedWarmstart", "PrimalBound", true, "Use a warm start for the NLP solver");

	settings->createSetting("NLPFixedMaxIters", "PrimalBound", 10, "Maximal number of iterations between calls", 0,
			OSINT_MAX);
	settings->createSetting("NLPFixedMaxElapsedTime", "PrimalBound", 5.0, "Maximal elapsed time between calls", 0,
			OSDBL_MAX);

	std::vector < std::string > enumAddPrimalPointAsInteriorPoint;
	enumAddPrimalPointAsInteriorPoint.push_back("Keep original");
	enumAddPrimalPointAsInteriorPoint.push_back("Keep both");
	enumAddPrimalPointAsInteriorPoint.push_back("Only new");
	enumAddPrimalPointAsInteriorPoint.push_back("Only average");
	settings->createSetting("AddPrimalBoundAsInteriorPoint", "Algorithm",
			static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepOriginal),
			"How to update interior point when new primal solution is found", enumAddPrimalPointAsInteriorPoint);
	enumAddPrimalPointAsInteriorPoint.clear();

	settings->createSetting("UseObjectiveLinesearch", "PrimalBound", true,
			"Use a linesearch to find a primal bound if objective nonlinear");
	std::vector < std::string > enumPrimalBoundNLPStartingPoint;
	enumPrimalBoundNLPStartingPoint.push_back("All solutions");
	enumPrimalBoundNLPStartingPoint.push_back("First solution");
	enumPrimalBoundNLPStartingPoint.push_back("All feasible solutions");
	enumPrimalBoundNLPStartingPoint.push_back("First and all feasible solutions");
	enumPrimalBoundNLPStartingPoint.push_back("Smallest constraint deviation solution");
	settings->createSetting("NLPFixedSource", "PrimalBound",
			static_cast<int>(ES_PrimalBoundNLPFixedPoint::FirstAndFeasibleSolutions), "Fixed MIP point source",
			enumPrimalBoundNLPStartingPoint);
	enumPrimalBoundNLPStartingPoint.clear();

	settings->createSetting("PrimalBoundNonlinearTolerance", "PrimalBound", 1e-8,
			"The nonlinear constraint tolerance for accepting primal bounds ");

	settings->createSetting("PrimalBoundLinearTolerance", "PrimalBound", 1e-9,
			"The linear constraint tolerance for accepting primal bounds ");

	processInfo->outputInfo("Initialization of settings complete.");

	settings->settingsInitialized = true;
}

void SHOTSolver::initializeDebugMode()
{
	FileUtil* fileUtil = new FileUtil();
	auto debugPath = settings->getStringSetting("DebugPath", "SHOTSolver");

	boost::filesystem::path debugDir(boost::filesystem::current_path() / debugPath);

	if (boost::filesystem::exists(debugDir))
	{
		processInfo->outputInfo("Debug directory " + debugPath + " already exists.");
	}
	else
	{
		if (boost::filesystem::create_directories(debugDir))
		{
			processInfo->outputInfo("Debug directory " + debugPath + " created.");
		}
		else
		{
			processInfo->outputWarning("Could not create debug directory.");
		}
	}

	boost::filesystem::path source(settings->getStringSetting("ProblemFile", "SHOTSolver"));
	boost::filesystem::copy_file(boost::filesystem::canonical(source), debugDir / source.filename(),
			boost::filesystem::copy_option::overwrite_if_exists);

	fileUtil->writeFileFromString(debugPath + "/options.xml", getOSol());

	delete fileUtil;
}
