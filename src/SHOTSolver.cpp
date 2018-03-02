#include "SHOTSolver.h"

SHOTSolver::SHOTSolver()
#ifdef HAS_GAMS
	: gms2os(NULL)
#endif
{
	initializeSettings();
	isProblemInitialized = false;
}

SHOTSolver::~SHOTSolver()
{
	if (isProblemInitialized)
	{
		delete solutionStrategy;
	}

#ifdef HAS_GAMS
	delete gms2os;
#endif
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
							(ENUM_OUTPUT_LEVEL)(Settings::getInstance().getIntSetting("Console.LogLevel", "Output") + 1));
	osoutput->SetPrintLevel("shotlogfile",
							(ENUM_OUTPUT_LEVEL)(Settings::getInstance().getIntSetting("File.LogLevel", "Output") + 1));

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
#ifdef HAS_GAMS
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
#endif
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
	Settings::getInstance().updateSetting("ProblemFile", "Output", problemFile.string());

	if (static_cast<ES_OutputDirectory>(Settings::getInstance().getIntSetting("OutputDirectory", "Output")) == ES_OutputDirectory::Program)
	{
		boost::filesystem::path debugPath(boost::filesystem::current_path());
		debugPath /= problemName;

		Settings::getInstance().updateSetting("Debug.Path", "Output", "problemdebug/" + problemName.string());
		Settings::getInstance().updateSetting("ResultPath", "Output", boost::filesystem::current_path().string());
	}
	else
	{
		boost::filesystem::path debugPath(problemPath);
		debugPath /= problemName;

		Settings::getInstance().updateSetting("Debug.Path", "Output", debugPath.string());
		Settings::getInstance().updateSetting("ResultPath", "Output", problemPath.string());
	}

	if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
		initializeDebugMode();

	ProcessInfo::getInstance().outputAlways(" Problem read from file \"" + fileName + "\"");
	bool status = this->setProblem(tmpInstance);

	return (status);
}

bool SHOTSolver::setProblem(OSInstance *osInstance)
{
	if (static_cast<ES_MIPSolver>(Settings::getInstance().getIntSetting("MIP.Solver", "Dual")) == ES_MIPSolver::Cbc)
	{
		if (UtilityFunctions::areAllVariablesReal(osInstance))
		{
			ProcessInfo::getInstance().outputSummary(" Using NLP solution strategy.");
			solutionStrategy = new SolutionStrategyNLP(osInstance);
			ProcessInfo::getInstance().usedSolutionStrategy = E_SolutionStrategy::NLP;
		}
		else
		{
			solutionStrategy = new SolutionStrategyMultiTree(osInstance);
			isProblemInitialized = true;
		}

		return (true);
	}

	bool useQuadraticObjective = (static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"))) == ES_QuadraticProblemStrategy::QuadraticObjective;
	bool useQuadraticConstraints = (static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"))) == ES_QuadraticProblemStrategy::QuadraticallyConstrained;

	if (useQuadraticObjective && UtilityFunctions::isObjectiveQuadratic(osInstance) && UtilityFunctions::areAllConstraintsLinear(osInstance))
	//MIQP problem
	{
		ProcessInfo::getInstance().outputSummary(" Using MIQP solution strategy.");
		solutionStrategy = new SolutionStrategyMIQCQP(osInstance);
		ProcessInfo::getInstance().usedSolutionStrategy = E_SolutionStrategy::MIQP;
	}
	//MIQCQP problem
	else if (useQuadraticConstraints && UtilityFunctions::areAllConstraintsQuadratic(osInstance))
	{
		ProcessInfo::getInstance().outputSummary(" Using MIQCQP solution strategy.");
		solutionStrategy = new SolutionStrategyMIQCQP(osInstance);
		ProcessInfo::getInstance().usedSolutionStrategy = E_SolutionStrategy::MIQCQP;
	}
	else if (UtilityFunctions::areAllVariablesReal(osInstance))
	{
		ProcessInfo::getInstance().outputSummary(" Using NLP solution strategy.");
		solutionStrategy = new SolutionStrategyNLP(osInstance);
		ProcessInfo::getInstance().usedSolutionStrategy = E_SolutionStrategy::NLP;
	}
	else
	{
		switch (static_cast<ES_TreeStrategy>(Settings::getInstance().getIntSetting("TreeStrategy", "Dual")))
		{
		case (ES_TreeStrategy::SingleTree):
			ProcessInfo::getInstance().outputSummary(" Using single-tree solution strategy.");
			solutionStrategy = new SolutionStrategySingleTree(osInstance);
			ProcessInfo::getInstance().usedSolutionStrategy = E_SolutionStrategy::SingleTree;
			break;
		case (ES_TreeStrategy::MultiTree):
			ProcessInfo::getInstance().outputSummary(" Using multi-tree solution strategy.");
			solutionStrategy = new SolutionStrategyMultiTree(osInstance);
			ProcessInfo::getInstance().usedSolutionStrategy = E_SolutionStrategy::MultiTree;
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

#ifdef HAS_GAMS
	if (result && gms2os != NULL)
	{
		gms2os->writeResult(ProcessInfo::getInstance());
	}
#endif

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
		ProcessInfo::getInstance().outputWarning("Warning! Settings have already been initialized. Ignoring new settings.");
		return;
	}

	std::string empty = "empty"; // Used to create empty string options

	ProcessInfo::getInstance().outputInfo("Starting initialization of settings:");

	// Dual strategy settings: ECP and ESH

	std::vector<std::string> enumHyperplanePointStrategy;
	enumHyperplanePointStrategy.push_back("ESH");
	enumHyperplanePointStrategy.push_back("ECP");
	Settings::getInstance().createSetting("CutStrategy", "Dual",
										  static_cast<int>(ES_HyperplaneCutStrategy::ESH), "Dual cut strategy",
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
										  "Iteration limit for minimax cutting plane solver", 1, OSINT_MAX);

	Settings::getInstance().createSetting("ESH.InteriorPoint.CuttingPlane.IterationLimitSubsolver", "Dual", 1000,
										  "Iteration limit for minimization subsolver", 0, OSINT_MAX);

	Settings::getInstance().createSetting("ESH.InteriorPoint.CuttingPlane.Reuse", "Dual", true,
										  "Reuse valid cutting planes in main dual model");

	Settings::getInstance().createSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceAbs", "Dual", 1.0,
										  "Absolute termination tolerance between LP and linesearch objective", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceRel", "Dual", 1.0,
										  "Relative termination tolerance between LP and linesearch objective", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("ESH.InteriorPoint.MinimaxObjectiveLowerBound", "Dual", -10.0e12,
										  "Lower bound for minimax objective variable", -OSDBL_MAX, 0);

	Settings::getInstance().createSetting("ESH.InteriorPoint.MinimaxObjectiveUpperBound", "Dual", 0.1,
										  "Upper bound for minimax objective variable", -OSDBL_MAX, OSDBL_MAX);

	// Dual strategy settings: Interior point search strategy

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
										  static_cast<int>(ES_RootsearchConstraintStrategy::IndividualConstraints),
										  "Perform root search on", enumLinesearchConstraintStrategy);
	enumLinesearchConstraintStrategy.clear();

	Settings::getInstance().createSetting("ESH.Linesearch.IndividualConstraints.Unique", "Dual", true, "Allow only one hyperplane per constraint per iteration");

	Settings::getInstance().createSetting("ESH.Linesearch.ConstraintTolerance", "Dual", 1e-8,
										  "Constraint tolerance for when not to add individual hyperplanes", 0, OSDBL_MAX);

	// Dual strategy settings: Fixed integer (NLP) strategy

	Settings::getInstance().createSetting("FixedInteger.ConstraintTolerance", "Dual", 0.00001,
										  "Constraint tolerance for fixed strategy", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("FixedInteger.MaxIterations", "Dual", 20,
										  "Max LP iterations for fixed strategy", 0, OSINT_MAX);

	Settings::getInstance().createSetting("FixedInteger.ObjectiveTolerance", "Dual", 0.001,
										  "Objective tolerance for fixed strategy", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("FixedInteger.Use", "Dual", true,
										  "Solve a fixed LP problem if integer-values have not changes in several MIP iterations");

	// Dual strategy settings: Hyperplane generation

	Settings::getInstance().createSetting("HyperplaneCuts.Delay", "Dual", true,
										  "Add hyperplane cuts to model only after optimal MIP solution");

	Settings::getInstance().createSetting("HyperplaneCuts.MaxPerIteration", "Dual", 200,
										  "Maximal number of hyperplanes to add per iteration", 0, OSINT_MAX);

	Settings::getInstance().createSetting("HyperplaneCuts.UseIntegerCuts", "Dual", true,
										  "Add integer cuts for infeasible integer-combinations for binary problems");

	Settings::getInstance().createSetting("HyperplaneCuts.UsePrimalObjectiveCut", "Dual", true,
										  "Add an objective cut in the primal solution");

	// Dual strategy settings: MIP solver

	Settings::getInstance().createSetting("MIP.CutOffTolerance", "Dual", 0.00001,
										  "An extra tolerance for the objective cutoff value (to prevent infeasible subproblems)", 0.0, OSDBL_MAX);

	std::vector<std::string> enumPresolve;
	enumPresolve.push_back("Never");
	enumPresolve.push_back("Once");
	enumPresolve.push_back("Always");
	Settings::getInstance().createSetting("MIP.Presolve.Frequency", "Dual", static_cast<int>(ES_MIPPresolveStrategy::Once),
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
										  "Time (s) without dual bound updates for forcing optimal MIP solution", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("MIP.SolutionLimit.IncreaseIterations", "Dual", 50,
										  "Max number of iterations between MIP solution limit increases", 0, OSINT_MAX);

	Settings::getInstance().createSetting("MIP.SolutionLimit.Initial", "Dual", 1, "Initial MIP solution limit", 1, OSINT_MAX);

	Settings::getInstance().createSetting("MIP.SolutionLimit.UpdateTolerance", "Dual", 0.001,
										  "The constraint tolerance for when to update MIP solution limit", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("MIP.SolutionPool.Capacity", "Dual", 100, "The maximum number of solutions in the solution pool", 0, OSINT_MAX);

	Settings::getInstance().createSetting("MIP.SolutionPool.Populate", "Dual", false,
										  "Try to populate the solution pool after MIP iteration termination", true);

	std::vector<std::string> enumMIPSolver;
	enumMIPSolver.push_back("Cplex");
	enumMIPSolver.push_back("Gurobi");
	enumMIPSolver.push_back("Cbc");
	Settings::getInstance().createSetting("MIP.Solver", "Dual", static_cast<int>(ES_MIPSolver::Cplex), "What MIP solver to use", enumMIPSolver);
	enumMIPSolver.clear();

	Settings::getInstance().createSetting("MIP.UpdateObjectiveBounds", "Dual", false, "Update nonlinear objective variable bounds to primal/dual bounds");

	// Dual strategy settings: Objective linesearch

	Settings::getInstance().createSetting("ObjectiveLinesearch.Use", "Dual", true, "Update the solution value for a nonlinear objective variable through a linesearch");

	// Dual strategy settings: Quadratic function strategy

	std::vector<std::string> enumQPStrategy;
	enumQPStrategy.push_back("All nonlinear");
	enumQPStrategy.push_back("Use quadratic objective");
	enumQPStrategy.push_back("Use quadratic constraints");
	Settings::getInstance().createSetting("QuadraticStrategy", "Dual", static_cast<int>(ES_QuadraticProblemStrategy::QuadraticObjective), "How to treat quadratic functions", enumQPStrategy);
	enumQPStrategy.clear();

	// Dual strategy settings: Relaxation strategies

	Settings::getInstance().createSetting("Relaxation.Frequency", "Dual", 0,
										  "The frequency to solve an LP problem: 0: Disable", 0, OSINT_MAX);

	Settings::getInstance().createSetting("Relaxation.IterationLimit", "Dual", 200, "The max number of relaxed LP problems to solve initially", 0, OSINT_MAX);

	Settings::getInstance().createSetting("Relaxation.MaxLazyConstraints", "Dual", 0,
										  "Max number of lazy constraints to add in relaxed solutions in single-tree strategy", 0, OSINT_MAX);

	Settings::getInstance().createSetting("Relaxation.TerminationTolerance", "Dual", 0.5,
										  "Time limit (s) when solving LP problems initially");

	Settings::getInstance().createSetting("Relaxation.TimeLimit", "Dual", 30.0, "Time limit (s) when solving LP problems initially", 0, OSDBL_MAX);

	// Dual strategy settings: Main tree strategy

	std::vector<std::string> enumSolutionStrategy;
	enumSolutionStrategy.push_back("Multi-tree");
	enumSolutionStrategy.push_back("Single-tree");
	Settings::getInstance().createSetting("TreeStrategy", "Dual", static_cast<int>(ES_TreeStrategy::SingleTree),
										  "The main strategy to use", enumSolutionStrategy);
	enumSolutionStrategy.clear();

	// Optimization model settings
	Settings::getInstance().createSetting("ContinuousVariable.EmptyLowerBound", "Model", -10.0e11, "Lower bound for continuous variables without bounds", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("ContinuousVariable.EmptyUpperBound", "Model", 10.0e11, "Upper bound for continuous variables without bounds", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("IntegerVariable.EmptyLowerBound", "Model", 0.0, "Lower bound for integer variables without bounds", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("IntegerVariable.EmptyUpperBound", "Model", 2.0e9, "Upper bound for integer variables without bounds", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("NonlinearObjectiveVariable.Bound", "Model", 10.0e11, "Max absolute bound for the auxiliary nonlinear objective variable", 0, OSDBL_MAX);

	// Logging and output settings
	std::vector<std::string> enumLogLevel;
	enumLogLevel.push_back("Error");
	enumLogLevel.push_back("Summary");
	enumLogLevel.push_back("Warning");
	enumLogLevel.push_back("Info");
	enumLogLevel.push_back("Debug");
	enumLogLevel.push_back("Trace");
	enumLogLevel.push_back("Detailed trace");
	Settings::getInstance().createSetting("Console.LogLevel", "Output",
										  static_cast<int>(ENUM_OUTPUT_LEVEL_summary) - 1, "Log level for console output", enumLogLevel);

	Settings::getInstance().createSetting("Debug.Enable", "Output", false, "Use debug functionality");

	Settings::getInstance().createSetting("Debug.Path", "Output", empty, "The path where to save the debug information", true);

	Settings::getInstance().createSetting("File.LogLevel", "Output", static_cast<int>(ENUM_OUTPUT_LEVEL_summary) - 1,
										  "Log level for file output", enumLogLevel);
	enumLogLevel.clear();

	Settings::getInstance().createSetting("Console.DualSolver.Show", "Output", false, "Show output from dual solver on console");

	Settings::getInstance().createSetting("Console.GAMS.Show", "Output", false, "Show GAMS output on console");

	std::vector<std::string> enumOutputDirectory;
	enumOutputDirectory.push_back("Problem directory");
	enumOutputDirectory.push_back("Program directory");
	Settings::getInstance().createSetting("OutputDirectory", "Output",
										  static_cast<int>(ES_OutputDirectory::Program), "Where to save the output files", enumOutputDirectory);
	enumOutputDirectory.clear();

	Settings::getInstance().createSetting("SaveAllSolutions", "Output", false, "Save all intermediate primal solutions to OSrL file");

	// Primal settings: Fixed integer strategy
	std::vector<std::string> enumPrimalNLPStrategy;
	enumPrimalNLPStrategy.push_back("Use each iteration");
	enumPrimalNLPStrategy.push_back("Based on iteration or time");
	enumPrimalNLPStrategy.push_back("Based on iteration or time, and for all feasible MIP solutions");

	Settings::getInstance().createSetting("FixedInteger.CallStrategy", "Primal",
										  static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions),
										  "When should the fixed strategy be used", enumPrimalNLPStrategy);
	enumPrimalNLPStrategy.clear();

	Settings::getInstance().createSetting("FixedInteger.CreateInfeasibilityCut", "Primal", true,
										  "Create a cut from an infeasible solution point");

	Settings::getInstance().createSetting("FixedInteger.Frequency.Dynamic", "Primal", true,
										  "Dynamically update the call frequency based on success");

	Settings::getInstance().createSetting("FixedInteger.Frequency.Iteration", "Primal", 10,
										  "Max number of iterations between calls", 0, OSINT_MAX);

	Settings::getInstance().createSetting("FixedInteger.Frequency.Time", "Primal", 5.0,
										  "Max duration (s) between calls", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("FixedInteger.DualPointGap.Relative", "Primal", 0.001,
										  "If the objective gap between the MIP point and dual solution is less than this the fixed strategy is activated", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("FixedInteger.IterationLimit", "Primal", 10000000, "Max number of iterations per call", 0, OSINT_MAX);

	std::vector<std::string> enumPrimalNLPSolver;
	enumPrimalNLPSolver.push_back("CuttingPlane");
	enumPrimalNLPSolver.push_back("Ipopt");
	enumPrimalNLPSolver.push_back("GAMS");

	Settings::getInstance().createSetting("FixedInteger.Solver", "Primal", static_cast<int>(ES_PrimalNLPSolver::Ipopt),
										  "NLP solver to use", enumPrimalNLPSolver);
	enumPrimalNLPSolver.clear();

	std::vector<std::string> enumPrimalBoundNLPStartingPoint;
	enumPrimalBoundNLPStartingPoint.push_back("All");
	enumPrimalBoundNLPStartingPoint.push_back("First");
	enumPrimalBoundNLPStartingPoint.push_back("All feasible");
	enumPrimalBoundNLPStartingPoint.push_back("First and all feasible");
	enumPrimalBoundNLPStartingPoint.push_back("With smallest constraint deviation");
	Settings::getInstance().createSetting("FixedInteger.Source", "Primal",
										  static_cast<int>(ES_PrimalNLPFixedPoint::FirstAndFeasibleSolutions), "Source of fixed MIP solution point",
										  enumPrimalBoundNLPStartingPoint);
	enumPrimalBoundNLPStartingPoint.clear();

	Settings::getInstance().createSetting("FixedInteger.TimeLimit", "Primal", 10.0,
										  "Time limit (s) per NLP problem", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("FixedInteger.Use", "Primal", true,
										  "Use the fixed integer primal strategy");

	Settings::getInstance().createSetting("FixedInteger.UsePresolveBounds", "Primal", false,
										  "Use variable bounds from MIP in NLP problems. Warning! Does not seem to work", true);

	Settings::getInstance().createSetting("FixedInteger.Warmstart", "Primal", true,
										  "Warm start the NLP solver");

	// Primal settings: linesearch

	Settings::getInstance().createSetting("Linesearch.Use", "Primal", true,
										  "Use a linesearch to find primal solutions");

	// Primal settings: tolerances for accepting primal solutions

	Settings::getInstance().createSetting("Tolerance.TrustLinearConstraintValues", "Primal", false,
										  "Trust that subsolvers (NLP, MIP) give primal solutions that respect linear constraints");

	Settings::getInstance().createSetting("Tolerance.Integer", "Primal", 1e-6,
										  "Integer tolerance for accepting primal solutions");

	Settings::getInstance().createSetting("Tolerance.LinearConstraint", "Primal", 1e-6,
										  "Linear constraint tolerance for accepting primal solutions");

	Settings::getInstance().createSetting("Tolerance.NonlinearConstraint", "Primal", 1e-6,
										  "Nonlinear constraint tolerance for accepting primal solutions");

	// Subsolver settings: Cplex

	Settings::getInstance().createSetting("Cplex.MemoryEmphasis", "Subsolver", 0, "Try to conserve memory when possible", 0, 1);

	Settings::getInstance().createSetting("Cplex.MIPEmphasis", "Subsolver", 0,
										  "Sets the MIP emphasis: 0: Balanced. 1: Feasibility. 2: Optimality. 3: Best bound. 4: Hidden feasible", 0, 4);

	Settings::getInstance().createSetting("Cplex.NodeFileInd", "Subsolver", 1,
										  "Where to store the node file: 0: No file. 1: Compressed in memory. 2: On disk. 3: Compressed on disk.", 0, 3);

	Settings::getInstance().createSetting("Cplex.NumericalEmphasis", "Subsolver", 0, "Emphasis on numerical stability", 0, 1);

	Settings::getInstance().createSetting("Cplex.ParallelMode", "Subsolver", 0,
										  "Sets the parallel optimization mode: -1: Opportunistic. 0: Automatic. 1: Deterministic.", -1, 1);

	Settings::getInstance().createSetting("Cplex.Probe", "Subsolver", 0,
										  "Sets the MIP probing level: -1: No probing. 0: Automatic. 1: Moderate. 2: Aggressive. 3: Very aggressive", -1, 3);

	Settings::getInstance().createSetting("Cplex.SolnPoolGap", "Subsolver", 1.0e+75,
										  "Sets the relative gap filter on objective values in the solution pool", 0, 1.0e+75);

	Settings::getInstance().createSetting("Cplex.SolnPoolIntensity", "Subsolver", 0,
										  "Controls how much time and memory should be used when filling the solution pool: 0: Automatic. 1: Mild. 2: Moderate. 3: Aggressive. 4: Very aggressive", 0, 4);

	Settings::getInstance().createSetting("Cplex.SolnPoolReplace", "Subsolver", 1,
										  "How to replace solutions in the solution pool when full: 0: Replace oldest. 1: Replace worst. 2: Find diverse.", 0, 2);

	Settings::getInstance().createSetting("Cplex.UseNewCallbackType", "Subsolver", false,
										  "Use the new callback type (vers. >12.8) with single-tree strategy (experimental)");

	std::string workdir = "/data/stuff/tmp/";
	Settings::getInstance().createSetting("Cplex.WorkDir", "Subsolver", workdir, "Directory for swap file");

	Settings::getInstance().createSetting("Cplex.WorkMem", "Subsolver", 30000.0,
										  "Memory limit for when to start swapping to disk", 0, 1.0e+75);

	// Subsolver settings: Gurobi
	Settings::getInstance().createSetting("Gurobi.ScaleFlag", "Subsolver", 1, "Controls model scaling: 0: Off. 1: Agressive. 2: Very agressive.", 0, 2);

	Settings::getInstance().createSetting("Gurobi.MIPFocus", "Subsolver", 0, "MIP focus: 0: Automatic. 1: Feasibility. 2: Optimality. 3: Best bound.", 0, 3);

	Settings::getInstance().createSetting("Gurobi.NumericFocus", "Subsolver", 0, "Numeric focus (higher number more careful): 0: Automatic. 3: Most careful.", 0, 3);

	// Subsolver settings: GAMS NLP

	std::string optfile = "";
	Settings::getInstance().createSetting("GAMS.NLP.OptionsFilename", "Subsolver", optfile,
										  "Options file for the NLP solver in GAMS");

	std::string solver = "conopt";
	Settings::getInstance().createSetting("GAMS.NLP.Solver", "Subsolver", solver, "NLP solver to use in GAMS");

	// Subsolver settings: Ipopt

	Settings::getInstance().createSetting("Ipopt.ConstraintViolationTolerance", "Subsolver", 1E-8,
										  "Constraint violation tolerance in Ipopt", -OSDBL_MAX, OSDBL_MAX);

	std::vector<std::string> enumIPOptSolver;
	enumIPOptSolver.push_back("ma27");
	enumIPOptSolver.push_back("ma57");
	enumIPOptSolver.push_back("ma86");
	enumIPOptSolver.push_back("ma97");
	enumIPOptSolver.push_back("mumps");
	Settings::getInstance().createSetting("Ipopt.LinearSolver", "Subsolver", static_cast<int>(ES_IpoptSolver::ma57),
										  "Ipopt linear subsolver", enumIPOptSolver);
	enumIPOptSolver.clear();

	Settings::getInstance().createSetting("Ipopt.MaxIterations", "Subsolver", 1000,
										  "Maximum number of iterations");

	Settings::getInstance().createSetting("Ipopt.RelativeConvergenceTolerance", "Subsolver", 1E-8,
										  "Relative convergence tolerance");

	// Subsolver settings: root searches

	Settings::getInstance().createSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver", 0.0,
										  "Epsilon constraint tolerance for root search", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("Rootsearch.MaxIterations", "Subsolver", 100, "Maximal root search iterations",
										  0, OSINT_MAX);

	std::vector<std::string> enumLinesearchMethod;
	enumLinesearchMethod.push_back("BoostTOMS748");
	enumLinesearchMethod.push_back("BoostBisection");
	enumLinesearchMethod.push_back("Bisection");
	Settings::getInstance().createSetting("Rootsearch.Method", "Subsolver",
										  static_cast<int>(ES_RootsearchMethod::BoostTOMS748), "Root search method to use", enumLinesearchMethod);
	enumLinesearchMethod.clear();

	Settings::getInstance().createSetting("Rootsearch.TerminationTolerance", "Subsolver", 1e-16,
										  "Epsilon lambda tolerance for root search", 0.0, OSDBL_MAX);

	// Subsolver settings: termination

	Settings::getInstance().createSetting("ConstraintTolerance", "Termination", 1e-8,
										  "Termination tolerance for nonlinear constraints", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("IterationLimit", "Termination", 200000, "Iteration limit for main strategy", 1,
										  OSINT_MAX);

	Settings::getInstance().createSetting("ObjectiveGap.Absolute", "Termination", 0.001,
										  "Absolute gap termination tolerance for objective function", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("ObjectiveGap.Relative", "Termination", 0.001,
										  "Relative gap termination tolerance for objective function", 0, OSDBL_MAX);

	Settings::getInstance().createSetting("ObjectiveStagnation.IterationLimit", "Termination", OSINT_MAX,
										  "Max number of iterations without significant objective value improvement", 0, OSINT_MAX);

	Settings::getInstance().createSetting("ObjectiveStagnation.Tolerance", "Termination", 0.000001,
										  "Objective value improvement tolerance", 0.0, OSDBL_MAX);

	Settings::getInstance().createSetting("TimeLimit", "Termination", 900.0, "Time limit (s) for solver", 0.0,
										  OSDBL_MAX);

	// Subsolver settings: termination

	Settings::getInstance().createSetting("ProblemFile", "Output", empty, "The filename of the problem.", true);

	Settings::getInstance().createSetting("ResultPath", "Output", empty,
										  "The path where to save the result information", true);

	Settings::getInstance().settingsInitialized = true;
	ProcessInfo::getInstance().outputInfo("Initialization of settings complete.");
}

void SHOTSolver::initializeDebugMode()
{
	auto debugPath = Settings::getInstance().getStringSetting("Debug.Path", "Output");
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

	boost::filesystem::path source(Settings::getInstance().getStringSetting("ProblemFile", "Output"));
	boost::filesystem::copy_file(boost::filesystem::canonical(source), debugDir / source.filename(),
								 boost::filesystem::copy_option::overwrite_if_exists);

	FileUtil *fileUtil = new FileUtil();

	fileUtil->writeFileFromString(debugPath + "/options.xml", getOSoL());

	delete fileUtil;
}

void SHOTSolver::verifySettings()
{
}