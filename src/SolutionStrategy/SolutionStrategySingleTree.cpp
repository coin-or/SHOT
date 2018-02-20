#include "SolutionStrategySingleTree.h"

SolutionStrategySingleTree::SolutionStrategySingleTree(OSInstance *osInstance)
{
	ProcessInfo::getInstance().createTimer("Reformulation", "Time spent reformulating problem");
	ProcessInfo::getInstance().createTimer("InteriorPointTotal", "Time spent finding interior point");

	auto solver = static_cast<ES_NLPSolver>(Settings::getInstance().getIntSetting("ESH.InteriorPoint.Solver", "Dual"));
	ProcessInfo::getInstance().createTimer("InteriorPoint", " - Solving interior point NLP problem");

	ProcessInfo::getInstance().createTimer("Subproblems", "Time spent solving subproblems");
	ProcessInfo::getInstance().createTimer("MIP", " - MIP problems");
	ProcessInfo::getInstance().createTimer("LP", " - Relaxed problems");
	ProcessInfo::getInstance().createTimer("PopulateSolutionPool", " - Populate solution pool");
	ProcessInfo::getInstance().createTimer("HyperplaneLinesearch", " - Linesearch");
	ProcessInfo::getInstance().createTimer("ObjectiveLinesearch", " - Objective linesearch");
	ProcessInfo::getInstance().createTimer("PrimalBoundTotal", " - Primal solution search");
	ProcessInfo::getInstance().createTimer("PrimalBoundSearchNLP", "    - NLP");
	ProcessInfo::getInstance().createTimer("PrimalBoundLinesearch", "    - Linesearch");
	ProcessInfo::getInstance().createTimer("PrimalBoundFixedLP", "    - Fixed LP");

	auto solverMIP = static_cast<ES_MIPSolver>(Settings::getInstance().getIntSetting("MIP.Solver", "Dual"));

	TaskBase *tFinalizeSolution = new TaskSequential();

	TaskBase *tInitMIPSolver = new TaskInitializeDualSolver(solverMIP, true);
	ProcessInfo::getInstance().tasks->addTask(tInitMIPSolver, "InitMIPSolver");

	auto MIPSolver = ProcessInfo::getInstance().MIPSolver;

	TaskBase *tInitOrigProblem = new TaskInitializeOriginalProblem(osInstance);
	ProcessInfo::getInstance().tasks->addTask(tInitOrigProblem, "InitOrigProb");

	TaskBase *tPrintProblemStats = new TaskPrintProblemStats();
	ProcessInfo::getInstance().tasks->addTask(tPrintProblemStats, "PrintProbStat");

	if (Settings::getInstance().getIntSetting("CutStrategy", "Dual") == (int)ES_HyperplaneCutStrategy::ESH && (ProcessInfo::getInstance().originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic || ProcessInfo::getInstance().originalProblem->getNumberOfNonlinearConstraints() != 0))
	{
		TaskBase *tFindIntPoint = new TaskFindInteriorPoint();
		ProcessInfo::getInstance().tasks->addTask(tFindIntPoint, "FindIntPoint");
	}

	TaskBase *tCreateDualProblem = new TaskCreateDualProblem(MIPSolver);
	ProcessInfo::getInstance().tasks->addTask(tCreateDualProblem, "CreateDualProblem");

	TaskBase *tInitializeLinesearch = new TaskInitializeLinesearch();
	ProcessInfo::getInstance().tasks->addTask(tInitializeLinesearch, "InitializeLinesearch");

	TaskBase *tInitializeIteration = new TaskInitializeIteration();
	ProcessInfo::getInstance().tasks->addTask(tInitializeIteration, "InitIter");

	TaskBase *tAddHPs = new TaskAddHyperplanes(MIPSolver);
	ProcessInfo::getInstance().tasks->addTask(tAddHPs, "AddHPs");

	TaskBase *tPrintIterHeader = new TaskPrintIterationHeader();

	ProcessInfo::getInstance().tasks->addTask(tPrintIterHeader, "PrintIterHeader");

	if (static_cast<ES_MIPPresolveStrategy>(Settings::getInstance().getIntSetting("MIP.Presolve.Frequency", "Dual")) != ES_MIPPresolveStrategy::Never)
	{
		TaskBase *tPresolve = new TaskPresolve(MIPSolver);
		ProcessInfo::getInstance().tasks->addTask(tPresolve, "Presolve");
	}

	TaskBase *tSolveIteration = new TaskSolveIteration(MIPSolver);
	ProcessInfo::getInstance().tasks->addTask(tSolveIteration, "SolveIter");
	
	TaskBase *tSelectPrimSolPool = new TaskSelectPrimalCandidatesFromSolutionPool();
	ProcessInfo::getInstance().tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
	dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

	TaskBase *tPrintIterReport = new TaskPrintIterationReport();
	ProcessInfo::getInstance().tasks->addTask(tPrintIterReport, "PrintIterReport");

	TaskBase *tCheckIterError = new TaskCheckIterationError("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckIterError, "CheckIterError");

	TaskBase *tCheckAbsGap = new TaskCheckAbsoluteGap("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");

	TaskBase *tCheckRelGap = new TaskCheckRelativeGap("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");

	//TaskBase *tCheckConstrTol = new TaskCheckConstraintTolerance("FinalizeSolution");
	//ProcessInfo::getInstance().tasks->addTask(tCheckConstrTol, "CheckConstrTol");

	//ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");
	//ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");

	if (Settings::getInstance().getIntSetting("FixedInteger.CallStrategy", "Primal") && ProcessInfo::getInstance().originalProblem->getNumberOfNonlinearConstraints() > 0 && ProcessInfo::getInstance().originalProblem->getNumberOfDiscreteVariables() > 0)
	{
		TaskBase *tSelectPrimFixedNLPSolPool = new TaskSelectPrimalFixedNLPPointsFromSolutionPool();
		ProcessInfo::getInstance().tasks->addTask(tSelectPrimFixedNLPSolPool, "SelectPrimFixedNLPSolPool");
		dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimFixedNLPSolPool);

		TaskBase *tSelectPrimNLPCheck = new TaskSelectPrimalCandidatesFromNLP();
		ProcessInfo::getInstance().tasks->addTask(tSelectPrimNLPCheck, "SelectPrimNLPCheck");
		dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimNLPCheck);

		ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");
		ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");
	}

	/*TaskBase *tCheckIterLim = new TaskCheckIterationLimit("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckIterLim, "CheckIterLim");

	TaskBase *tCheckTimeLim = new TaskCheckTimeLimit("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckTimeLim, "CheckTimeLim");*/

	TaskBase *tPrintBoundReport = new TaskPrintSolutionBoundReport();
	ProcessInfo::getInstance().tasks->addTask(tPrintBoundReport, "PrintBoundReport");

	ProcessInfo::getInstance().tasks->addTask(tFinalizeSolution, "FinalizeSolution");

	TaskBase *tPrintSol = new TaskPrintSolution();
	ProcessInfo::getInstance().tasks->addTask(tPrintSol, "PrintSol");
}

SolutionStrategySingleTree::~SolutionStrategySingleTree()
{
	// TODO Auto-generated destructor stub
}

bool SolutionStrategySingleTree::solveProblem()
{
	TaskBase *nextTask = new TaskBase;

	while (ProcessInfo::getInstance().tasks->getNextTask(nextTask))
	{
		ProcessInfo::getInstance().outputInfo("┌─── Started task:  " + nextTask->getType());
		nextTask->run();
		ProcessInfo::getInstance().outputInfo("└─── Finished task: " + nextTask->getType());
	}

	ProcessInfo::getInstance().tasks->clearTasks();

	return (true);
}

void SolutionStrategySingleTree::initializeStrategy()
{
}
