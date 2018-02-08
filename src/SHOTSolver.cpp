#include "SHOTSolver.h"

SHOTSolver::SHOTSolver() : gms2os(NULL)
{
	initializeSettings();
	isProblemInitialized = false;
}

SHOTSolver::~SHOTSolver()
{
	if (isProblemInitialized)
		delete solutionStrategy;
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
	catch (const ErrorClass &eclass)
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
	if (!boost::filesystem::exists(fileName))
	{
		ProcessInfo::getInstance().outputError("Problem file \"" + fileName + "\" does not exist.");

		return (false);
	}

	boost::filesystem::path problemFile(fileName);

	if (!problemFile.has_extension())
	{
		ProcessInfo::getInstance().outputError("Problem file \"" + fileName + "\" does not specify a file extension.");

		return (false);
	}

	OSInstance *tmpInstance;

	boost::filesystem::path problemExtension = problemFile.extension();
	boost::filesystem::path problemPath = problemFile.parent_path();

	try
	{
		if (problemExtension == ".osil" || problemExtension == ".xml")
		{
			FileUtil *fileUtil = new FileUtil();
			std::string fileContents = fileUtil->getFileAsString(fileName.c_str());
			delete fileUtil;

			OSiLReader *osilreader = new OSiLReader();
			tmpInstance = osilreader->readOSiL(fileContents);

			//delete osilreader;
			//TODO: Why can't osilreader be deleted
		}
		else if (problemExtension == ".nl")
		{
			OSnl2OS *nl2os = new OSnl2OS();
			nl2os->readNl(fileName);
			nl2os->createOSObjects();
			tmpInstance = nl2os->osinstance;
		}
		else if (problemExtension == ".gms")
		{
			assert(gms2os == NULL);
			gms2os = new GAMS2OS();
			gms2os->readGms(fileName);
			gms2os->createOSObjects();

			tmpInstance = gms2os->osinstance;
		}
		else if (problemExtension == ".dat")
		{
			assert(gms2os == NULL);
			gms2os = new GAMS2OS();
			gms2os->readCntr(fileName);
			gms2os->createOSObjects();
			tmpInstance = gms2os->osinstance;
		}
		else
		{
			ProcessInfo::getInstance().outputError("Wrong filetype specified.");

			delete tmpInstance;

			return (false);
		}

		tmpInstance->instanceHeader->source = fileName;
	}
	catch (const ErrorClass &eclass)
	{
		ProcessInfo::getInstance().outputError("Error when reading problem from \"" + fileName + "\"", eclass.errormsg);

		delete tmpInstance;

		return (false);
	}

	//Removes path
	boost::filesystem::path problemName = problemFile.stem();

	tmpInstance->setInstanceName(problemName.string());
	Settings::getInstance().updateSetting("ProblemFile", "SHOTSolver", problemFile.string());

	if (static_cast<ES_OutputDirectory>(Settings::getInstance().getIntSetting("OutputDirectory", "SHOT")) == ES_OutputDirectory::Program)
	{
		boost::filesystem::path debugPath(boost::filesystem::current_path());
		debugPath /= problemName;

		Settings::getInstance().updateSetting("DebugPath", "SHOTSolver", "problemdebug/" + problemName.string());
		Settings::getInstance().updateSetting("ResultPath", "SHOTSolver", boost::filesystem::current_path().string());
	}
	else
	{
		boost::filesystem::path debugPath(problemPath);
		debugPath /= problemName;

		Settings::getInstance().updateSetting("DebugPath", "SHOTSolver", debugPath.string());
		Settings::getInstance().updateSetting("ResultPath", "SHOTSolver", problemPath.string());
	}

	if (Settings::getInstance().getBoolSetting("Debug", "SHOTSolver"))
		initializeDebugMode();

	bool status = this->setProblem(tmpInstance);

	return (status);
}

bool SHOTSolver::setProblem(OSInstance *osInstance)
{
	if (static_cast<ES_MIPSolver>(Settings::getInstance().getIntSetting("MIP.Solver", "Dual")) == ES_MIPSolver::Cbc)
	{
		solutionStrategy = new SolutionStrategyNormal(osInstance);
		isProblemInitialized = true;

		return (true);
	}

	bool useQuadraticObjective = (static_cast<ES_QPStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"))) == ES_QPStrategy::QuadraticObjective;
	bool useQuadraticConstraints = (static_cast<ES_QPStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"))) == ES_QPStrategy::QuadraticallyConstrained;
	if (useQuadraticObjective && UtilityFunctions::isObjectiveQuadratic(osInstance) && UtilityFunctions::areAllConstraintsLinear(osInstance))
	//MIQP problem
	{
		solutionStrategy = new SolutionStrategyMIQCQP(osInstance);
	}
	//MIQCQP problem
	else if (useQuadraticConstraints && UtilityFunctions::areAllConstraintsQuadratic(osInstance))
	{
		solutionStrategy = new SolutionStrategyMIQCQP(osInstance);
	}
	else if (UtilityFunctions::areAllVariablesReal(osInstance))
	{
		solutionStrategy = new SolutionStrategyNLP(osInstance);
	}
	else
	{
		switch (static_cast<ES_SolutionStrategy>(Settings::getInstance().getIntSetting("TreeStrategy ", "Dual")))
		{
		case (ES_SolutionStrategy::SingleTree):
			solutionStrategy = new SolutionStrategyLazy(osInstance);
			break;
		case (ES_SolutionStrategy::MultiTree):
			solutionStrategy = new SolutionStrategyNormal(osInstance);
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
	if (!Settings::getInstance().settingsInitialized)
		initializeSettings();

	return (Settings::getInstance().getSettingsInOSolFormat());
}

std::string SHOTSolver::getGAMSOptFile()
{
	if (!Settings::getInstance().settingsInitialized)
		initializeSettings();

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

	std::vector<std::string> enumHyperplanePointStrategy;
	enumHyperplanePointStrategy.push_back("ESH");
	enumHyperplanePointStrategy.push_back("ECP");
	Settings::getInstance().createSetting("CutStrategy", "Dual",
										  static_cast<int>(ES_HyperplanePointStrategy::ESH), "Dual cut strategy",
										  enumHyperplanePointStrategy);
	enumHyperplanePointStrategy.clear();

	Settings::getInstance().createSetting("ECP.ConstraintSelectionFactor", "Dual", 0.0,
										  "The fraction of violated constraints to generate cutting planes for",
										  0.0, 1.0);

	Settings::getInstance().createSetting("ESH.InteriorPoint.CuttingPlane.BitPrecision", "Dual", 8,
										  "Required termination bit precision for minimization subsolver", 1, 64, true);

	Settings::getInstance().createSetting("ESH.InteriorPoint.CuttingPlane.ConstraintSelectionTolerance", "Dual", 0.05,
										  "Tolerance when selecting the most constraint with largest deviation", 0.0, 1.0);

	Settings::getInstance().createSetting("ESH.InteriorPoint.CuttingPlane.IterationLimit", "Dual", 2000,
										  "Iteration limit for minimax cutting plane solver", 0, OSINT_MAX);

	Settings::getInstance().createSetting("ESH.InteriorPoint.CuttingPlane.IterationLimitSubsolver", "Dual", 1000,
										  "Iteration limit for minimization subsolver", 0, OSINT_MAX);

	Settings::getInstance().createSetting("ESH.InteriorPoint.CuttingPlane.Reuse", "Dual", true,
										  "Reuse valid cutting planes in main dual model");

	Settings::getInstance().createSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceAbs", "Dual", 1.0,
										  "Absolute termination tolerance between LP and linesearch objective", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceRel", "Dual", 1.0,
										  "Relative termination tolerance between LP and linesearch objective", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("ESH.InteriorPoint.MinimaxObjectiveLowerBound", "Dual", -10000000000.0,
										  "Lower bound for minimax objective variable", -OSDBL_MAX, 0);

	Settings::getInstance().createSetting("ESH.InteriorPoint.MinimaxObjectiveUpperBound", "Dual", 0.1,
										  "Upper bound for minimax objective variable", -OSDBL_MAX, OSDBL_MAX);

	std::vector<std::string> enumNLPSolver;
	enumNLPSolver.push_back("Cutting plane minimax");
	enumNLPSolver.push_back("Ipopt minimax");
	enumNLPSolver.push_back("Ipopt relaxed");
	enumNLPSolver.push_back("Ipopt minimax and relaxed");

	Settings::getInstance().createSetting("ESH.InteriorPoint.Solver", "Dual",
										  static_cast<int>(ES_NLPSolver::CuttingPlaneMiniMax), "NLP solver", enumNLPSolver);
	enumNLPSolver.clear();

	std::vector<std::string> enumAddPrimalPointAsInteriorPoint;
	enumAddPrimalPointAsInteriorPoint.push_back("No");
	enumAddPrimalPointAsInteriorPoint.push_back("Add as new");
	enumAddPrimalPointAsInteriorPoint.push_back("Replace old");
	enumAddPrimalPointAsInteriorPoint.push_back("Use avarage");
	Settings::getInstance().createSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual",
										  static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepOriginal),
										  "Utilize primal solution as interior point", enumAddPrimalPointAsInteriorPoint);
	enumAddPrimalPointAsInteriorPoint.clear();

	Settings::getInstance().createSetting("ESH.Linesearch.ConstraintFactor", "Dual", 0.5,
										  "No linesearch on a constraint if its value is less than this factor of the maximum", 1e-6,
										  1.0);

	std::vector<std::string> enumLinesearchConstraintStrategy;
	enumLinesearchConstraintStrategy.push_back("Max function");
	enumLinesearchConstraintStrategy.push_back("Individual constraints");
	Settings::getInstance().createSetting("ESH.Linesearch.ConstraintStrategy", "Dual",
										  static_cast<int>(ES_LinesearchConstraintStrategy::IndividualConstraints),
										  "Perform root search on", enumLinesearchConstraintStrategy);
	enumLinesearchConstraintStrategy.clear();

	Settings::getInstance().createSetting("ESH.Linesearch.ConstraintTolerance", "Dual", 1e-8,
										  "Constraint tolerance for when not to add individual hyperplanes", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("FixedInteger.ConstraintTolerance", "Dual", 0.00001,
										  "Constraint tolerance for fixed strategy", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("FixedInteger.MaxIterations", "Dual", 20,
										  "Max LP iterations for fixed strategy", 0, OSINT_MAX);

	Settings::getInstance().createSetting("FixedInteger.ObjectiveTolerance", "Dual", 0.001,
										  "Objective tolerance for fixed strategy", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("FixedInteger.Use", "Dual", true,
										  "Solve a fixed LP problem if integer-values have not changes in several MIP iterations");

	Settings::getInstance().createSetting("HyperplaneCuts.Delay", "Dual", true,
										  "Add hyperplane cuts to model only after optimal MILP solution");

	Settings::getInstance().createSetting("HyperplaneCuts.MaxPerIteration", "Dual", 200,
										  "Maximal number of hyperplanes to add per iteration", 0, OSINT_MAX);

	Settings::getInstance().createSetting("HyperplaneCuts.UseIntegerCuts", "Dual", true,
										  "Add integer cuts for infeasible integer-combinations for binary problems");

	Settings::getInstance().createSetting("HyperplaneCuts.UsePrimalObjectiveCut", "Dual", true,
										  "Add an objective cut in the primal solution");

	Settings::getInstance().createSetting("MIP.CutOffTolerance", "Dual", 0.0001,
										  "An extra tolerance for the objective cutoff value (to prevent infeasible subproblems)", 0.0, OSDBL_MAX);

	std::vector<std::string> enumPresolve;
	enumPresolve.push_back("Never");
	enumPresolve.push_back("Once");
	enumPresolve.push_back("Always");
	Settings::getInstance().createSetting("MIP.Presolve.Frequency", "Dual", static_cast<int>(ES_PresolveStrategy::Once),
										  "When to call the MIP presolve", enumPresolve);
	enumPresolve.clear();

	Settings::getInstance().createSetting("MIP.Presolve.RemoveRedundantConstraints", "Dual", false,
										  "Remove redundant constraints (as determined by presolve)");

	Settings::getInstance().createSetting("MIP.Presolve.UpdateObtainedBounds", "Dual", true,
										  "Update bounds (from presolve) to the MIP model");

	Settings::getInstance().createSetting("MIP.NumberOfThreads", "Dual", 7, "Number of threads to use in MIP solver: 0: Automatic", 0, 999);

	Settings::getInstance().createSetting("MIP.SolutionLimit.ForceOptimal.Iteration", "Dual", 10000,
										  "Iterations without dual bound updates for forcing optimal MIP solution", 0, OSINT_MAX);

	Settings::getInstance().createSetting("MIP.SolutionLimit.ForceOptimal.Time", "Dual", 1000.0,
										  "Time without dual bound updates for forcing optimal MIP solution", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("MIP.SolutionLimit.IncreaseIterations", "Dual", 50,
										  "Max number of iterations between MIP solution limit increases", 0, OSINT_MAX);

	Settings::getInstance().createSetting("MIP.SolutionLimit.Initial", "Dual", 1, "Initial MIP solution limit", 1, OSINT_MAX);

	Settings::getInstance().createSetting("MIP.SolutionLimit.UpdateTolerance", "Dual", 0.001,
										  "The constraint tolerance for when to update MIP solution limit", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("MIP.SolutionPool.Capacity", "Dual", 100, "The maximum number of solutions in the solution pool", 0, OSINT_MAX);

	Settings::getInstance().createSetting("MIP.SolutionPool.Populate", "Dual", false,
										  "Try to populate the solution pool after MIP iteration termination", true);

	std::vector<std::string> enumMILPSolver;
	enumMILPSolver.push_back("Cplex");
	enumMILPSolver.push_back("Gurobi");
	enumMILPSolver.push_back("Cbc");
	Settings::getInstance().createSetting("MIP.Solver", "Dual", static_cast<int>(ES_MIPSolver::Cplex), "What MIP solver to use", enumMILPSolver);
	enumMILPSolver.clear();

	Settings::getInstance().createSetting("MIP.UpdateObjectiveBounds", "Dual", false, "Update nonlinear objective variable bounds to primal/dual bounds");

	Settings::getInstance().createSetting("ObjectiveLinesearch.Use", "Dual", true, "Update the solution value for a nonlinear objective variable through a linesearch");

	Settings::getInstance().createSetting("Relaxation.Frequency", "Dual", 0,
										  "The frequency to solve an LP problem: 0: Disable", 0, OSINT_MAX);

	Settings::getInstance().createSetting("Relaxation.IterationLimit", "Dual", 200, "The max number of relaxed LP problems to solve initially", 0, OSINT_MAX);

	Settings::getInstance().createSetting("Relaxation.MaxLazyConstraints", "Dual", 100,
										  "Max number of lazy constraints to add in relaxed solutions in single-tree strategy", 0, OSINT_MAX);

	Settings::getInstance().createSetting("Relaxation.TerminationTolerance", "Dual", 0.5,
										  "Time limit when solving LP problems initially");

	Settings::getInstance().createSetting("Relaxation.TimeLimit", "Dual", 30.0, "Time limit when solving LP problems initially", 0, OSDBL_MAX);

	std::vector<std::string> enumSolutionStrategy;
	enumSolutionStrategy.push_back("Multi-tree");
	enumSolutionStrategy.push_back("Single-tree");
	Settings::getInstance().createSetting("TreeStrategy ", "Dual", static_cast<int>(ES_SolutionStrategy::SingleTree),
										  "The main strategy to use", enumSolutionStrategy);
	enumSolutionStrategy.clear();

	// Logging setting
	std::vector<std::string> enumLogLevel;
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

	std::vector<std::string> enumOutputDirectory;
	enumOutputDirectory.push_back("Problem directory");
	enumOutputDirectory.push_back("Program directory");
	Settings::getInstance().createSetting("OutputDirectory", "SHOT",
										  static_cast<int>(ES_OutputDirectory::Program), "Where to save the output files", enumOutputDirectory);

	Settings::getInstance().createSetting("SaveAllPrimalSolutions", "SHOTSolver", false,
										  "Should all intermediate solutions (primal and dual) be saved.");

	// Relaxation
	Settings::getInstance().createSetting("ObjectiveStagnationIterationLimit", "Algorithm", 100,
										  "Max number of iterations without objective function improvement", 0, OSINT_MAX);
	Settings::getInstance().createSetting("ObjectiveStagnationTolerance", "Algorithm", 0.000001,
										  "Objective function improvement tolerance", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("ConstrTermTolMILP", "Algorithm", 1e-8,
										  "Final termination tolerance for constraints", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("GapTermTolAbsolute", "Algorithm", 0.001,
										  "Absolute gap tolerance for objective function", 0, OSDBL_MAX);
	Settings::getInstance().createSetting("GapTermTolRelative", "Algorithm", 0.001,
										  "Relative gap tolerance for objective function", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("TimeLimit", "Algorithm", 900.0, "Time limit in seconds for solver", 0.0,
										  OSDBL_MAX);
	Settings::getInstance().createSetting("IterLimitMILP", "Algorithm", 2000, "MILP iteration limit for solver", 1,
										  OSINT_MAX);

	std::vector<std::string> enumIPOptSolver;
	enumIPOptSolver.push_back("ma27");
	enumIPOptSolver.push_back("ma57");
	enumIPOptSolver.push_back("ma86");
	enumIPOptSolver.push_back("ma97");
	enumIPOptSolver.push_back("mumps");
	Settings::getInstance().createSetting("IpoptSolver", "Ipopt", static_cast<int>(ES_IPOptSolver::mumps),
										  "Linear subsolver in Ipopt", enumIPOptSolver);
	enumIPOptSolver.clear();

	Settings::getInstance().createSetting("ToleranceInteriorPointStrategy", "Ipopt", 1E-8,
										  "Relative convergence tolerance in interior point search strategy");
	Settings::getInstance().createSetting("MaxIterInteriorPointStrategy", "Ipopt", 1000,
										  "Maximum number of iterations in interior point search strategy");
	Settings::getInstance().createSetting("ConstraintToleranceInteriorPointStrategy", "Ipopt", 1E-8,
										  "Constraint violation tolerance", -OSDBL_MAX, OSDBL_MAX);

	Settings::getInstance().createSetting("UsePresolveBoundsForPrimalNLP", "Presolve", false,
										  "Warning! Does not seem to work. Use updated bounds from the MIP solver when solving primal NLP problems", true);

	//Settings::getInstance().createSetting("UseQuadraticProgramming", "Algorithm", true, "Solve QP problems if objective function is quadratic,");

	std::vector<std::string> enumQPStrategy;
	enumQPStrategy.push_back("All nonlinear");
	enumQPStrategy.push_back("Use quadratic objective");
	enumQPStrategy.push_back("Use quadratic constraints");
	Settings::getInstance().createSetting("QuadraticStrategy", "Dual", static_cast<int>(ES_QPStrategy::QuadraticObjective), "How to treat quadratic functions", enumQPStrategy);

	// CPLEX
	Settings::getInstance().createSetting("UseNewCallbackType", "CPLEX", false,
										  "Use the new callback introduced in CPLEX 12.8 with the lazy strategy");

	Settings::getInstance().createSetting("SolnPoolIntensity", "CPLEX", 0,
										  "0 = automatic, 1 = mild, 2 = moderate, 3 = aggressive, 4 = very aggressive", 0, 4);
	Settings::getInstance().createSetting("SolnPoolReplace", "CPLEX", 1,
										  "0 = replace oldest solution, 1 = replace worst solution, 2 = find diverse solutions", 0, 2);

	Settings::getInstance().createSetting("SolnPoolGap", "CPLEX", 1.0e+75,
										  "The relative tolerance on the objective values in the solution pool", 0, 1.0e+75);
	Settings::getInstance().createSetting("MIPEmphasis", "CPLEX", 0,
										  "0 = balanced, 1 = feasibility, 2 = optimality, 3 = best bound, 4 = hidden feasible", 0, 4);

	Settings::getInstance().createSetting("Probe", "CPLEX", 0,
										  "-1 = no probing, 0 = automatic, 1 = moderate, 2 = aggressive, 3 = very aggressive", -1, 3);

	Settings::getInstance().createSetting("ParallelMode", "CPLEX", 0,
										  "-1 = opportunistic, 0 = automatic, 1 = deterministic", -1, 1);

	// Linesearch
	std::vector<std::string> enumLinesearchMethod;
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

	std::string empty = "empty";

	Settings::getInstance().createSetting("ProblemFile", "SHOTSolver", empty, "The filename of the problem.", true);
	Settings::getInstance().createSetting("DebugPath", "SHOTSolver", empty,
										  "The path where to save the debug information", true);
	Settings::getInstance().createSetting("ResultPath", "SHOTSolver", empty,
										  "The path where to save the result information", true);
	Settings::getInstance().createSetting("Debug", "SHOTSolver", false, "Use debug functionality");

	// Primal bound
	Settings::getInstance().createSetting("PrimalStrategyLinesearch", "PrimalBound", true,
										  "Use the dedicated linesearch method for finding primal solutions");

	Settings::getInstance().createSetting("PrimalStrategyFixedNLP", "PrimalBound", true,
										  "Solve integer-fixed NLP problems for finding primal solutions");

	std::vector<std::string> enumPrimalNLPSolver;
	enumPrimalNLPSolver.push_back("CuttingPlane");
	enumPrimalNLPSolver.push_back("Ipopt");
	enumPrimalNLPSolver.push_back("GAMS");

	Settings::getInstance().createSetting("PrimalNLPSolver", "PrimalBound", static_cast<int>(ES_PrimalNLPSolver::IPOpt),
										  "NLP solver from fixed NLP primal bound strategy.", enumPrimalNLPSolver);
	enumPrimalNLPSolver.clear();

	std::vector<std::string> enumPrimalNLPStrategy;
	enumPrimalNLPStrategy.push_back("Use each iteration");
	enumPrimalNLPStrategy.push_back("Based on iteration or time");
	enumPrimalNLPStrategy.push_back("Based on iteration or time, and for all feasible MIP solutions");

	Settings::getInstance().createSetting("NLPFixedStrategy", "PrimalBound",
										  static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions),
										  "How to use fixed NLP primal calls", enumPrimalNLPStrategy);
	enumPrimalNLPStrategy.clear();

	Settings::getInstance().createSetting("NLPFixedWarmstart", "PrimalBound", true,
										  "Use a warm start for the NLP solver");

	Settings::getInstance().createSetting("NLPFixedUpdateItersAndTime", "PrimalBound", true,
										  "Automatically change call frequency based on success");
	Settings::getInstance().createSetting("NLPFixedMaxIters", "PrimalBound", 10,
										  "Maximal number of iterations between calls", 0, OSINT_MAX);
	Settings::getInstance().createSetting("NLPFixedMaxElapsedTime", "PrimalBound", 5.0,
										  "Maximal elapsed time between calls", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("NLPFixedCreateCutFromInfeasible", "PrimalBound", true,
										  "If the fixed NLP problem is infeasible, utilize the solution to create a cut.");

	Settings::getInstance().createSetting("NLPTimeLimit", "PrimalBound", 10.0,
										  "Maximal time allowed per fixed NLP problem", 0, OSDBL_MAX);
	Settings::getInstance().createSetting("NLPIterLimit", "PrimalBound", 10000000,
										  "Maximum number of iterations allowed per fixed NLP problem", 0, OSINT_MAX);

	std::string solver = "conopt";
	std::string optfile = "";

	Settings::getInstance().createSetting("NLPSolver", "GAMS", solver, "Name of the NLP solver to use in GAMS");
	Settings::getInstance().createSetting("NLPOptionsFile", "GAMS", optfile,
										  "Name of the options file for the NLP solver in GAMS");
	Settings::getInstance().createSetting("ShowOutput", "GAMS", false, "Show GAMS output");

	std::vector<std::string> enumPrimalBoundNLPStartingPoint;
	enumPrimalBoundNLPStartingPoint.push_back("All solutions");
	enumPrimalBoundNLPStartingPoint.push_back("First solution");
	enumPrimalBoundNLPStartingPoint.push_back("All feasible solutions");
	enumPrimalBoundNLPStartingPoint.push_back("First and all feasible solutions");
	enumPrimalBoundNLPStartingPoint.push_back("Smallest constraint deviation solution");
	Settings::getInstance().createSetting("NLPFixedSource", "PrimalBound",
										  static_cast<int>(ES_PrimalBoundNLPFixedPoint::FirstAndFeasibleSolutions), "Fixed MIP point source",
										  enumPrimalBoundNLPStartingPoint);
	enumPrimalBoundNLPStartingPoint.clear();

	Settings::getInstance().createSetting("PrimalBoundNonlinearTolerance", "PrimalBound", 1e-6,
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
	boost::filesystem::path debugDir(debugPath);

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

	FileUtil *fileUtil = new FileUtil();

	fileUtil->writeFileFromString(debugPath + "/options.xml", getOSoL());

	delete fileUtil;
}
