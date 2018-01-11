#include "SolutionStrategyLazy.h"

SolutionStrategyLazy::SolutionStrategyLazy(OSInstance* osInstance)
{
	ProcessInfo::getInstance().createTimer("Reformulation", "Time spent reformulating problem");
	ProcessInfo::getInstance().createTimer("InteriorPointTotal", "Time spent finding interior point");

	auto solver = static_cast<ES_NLPSolver>(Settings::getInstance().getIntSetting("InteriorPointSolver",
			"InteriorPoint"));
	ProcessInfo::getInstance().createTimer("InteriorPoint", " - Solving interior point NLP problem");

	ProcessInfo::getInstance().createTimer("Subproblems", "Time spent solving subproblems");
	ProcessInfo::getInstance().createTimer("MILP", " - MIP problems");
	ProcessInfo::getInstance().createTimer("LP", " - Relaxed problems");
	ProcessInfo::getInstance().createTimer("PopulateSolutionPool", " - Populate solution pool");
	ProcessInfo::getInstance().createTimer("HyperplaneLinesearch", " - Linesearch");
	ProcessInfo::getInstance().createTimer("ObjectiveLinesearch", " - Objective linesearch");
	ProcessInfo::getInstance().createTimer("PrimalBoundTotal", " - Primal solution search");
	ProcessInfo::getInstance().createTimer("PrimalBoundSearchNLP", "    - NLP");
	ProcessInfo::getInstance().createTimer("PrimalBoundLinesearch", "    - Linesearch");
	ProcessInfo::getInstance().createTimer("PrimalBoundFixedLP", "    - Fixed LP");

	auto solverMILP = static_cast<ES_MILPSolver>(Settings::getInstance().getIntSetting("MILPSolver", "MILP"));

	TaskBase *tFinalizeSolution = new TaskSequential();

	TaskBase *tInitMILPSolver = new TaskInitializeMILPSolver(osInstance);
	ProcessInfo::getInstance().tasks->addTask(tInitMILPSolver, "InitMILPSolver");

	auto MILPSolver = ProcessInfo::getInstance().MILPSolver;

	TaskBase *tInitOrigProblem = new TaskInitializeOriginalProblem(osInstance);
	ProcessInfo::getInstance().tasks->addTask(tInitOrigProblem, "InitOrigProb");

	TaskBase *tPrintProblemStats = new TaskPrintProblemStats();
	ProcessInfo::getInstance().tasks->addTask(tPrintProblemStats, "PrintProbStat");

	if (Settings::getInstance().getIntSetting("HyperplanePointStrategy", "Algorithm")
			== (int) ES_HyperplanePointStrategy::ESH
			&& (ProcessInfo::getInstance().originalProblem->getObjectiveFunctionType()
					!= E_ObjectiveFunctionType::Quadratic
					|| ProcessInfo::getInstance().originalProblem->getNumberOfNonlinearConstraints() != 0))
	{
		TaskBase *tFindIntPoint = new TaskFindInteriorPoint();
		ProcessInfo::getInstance().tasks->addTask(tFindIntPoint, "FindIntPoint");
	}

	TaskBase *tCreateMILPProblem = new TaskCreateMILPProblem(MILPSolver);
	ProcessInfo::getInstance().tasks->addTask(tCreateMILPProblem, "CreateMILPProblem");

	TaskBase *tInitializeLinesearch = new TaskInitializeLinesearch();
	ProcessInfo::getInstance().tasks->addTask(tInitializeLinesearch, "InitializeLinesearch");

	TaskBase *tInitializeIteration = new TaskInitializeIteration();
	ProcessInfo::getInstance().tasks->addTask(tInitializeIteration, "InitIter");

	TaskBase *tAddHPs = new TaskAddHyperplanes(MILPSolver);
	ProcessInfo::getInstance().tasks->addTask(tAddHPs, "AddHPs");

	TaskBase *tPrintIterHeader = new TaskPrintIterationHeader();

	ProcessInfo::getInstance().tasks->addTask(tPrintIterHeader, "PrintIterHeader");

	if (static_cast<ES_PresolveStrategy>(Settings::getInstance().getIntSetting("PresolveStrategy", "Presolve"))
			!= ES_PresolveStrategy::Never)
	{
		TaskBase *tPresolve = new TaskPresolve(MILPSolver);
		ProcessInfo::getInstance().tasks->addTask(tPresolve, "Presolve");
	}

	TaskBase *tSolveIteration = new TaskSolveIteration(MILPSolver);
	ProcessInfo::getInstance().tasks->addTask(tSolveIteration, "SolveIter");

	TaskBase *tPrintIterReport = new TaskPrintIterationReport();
	ProcessInfo::getInstance().tasks->addTask(tPrintIterReport, "PrintIterReport");

	TaskBase *tCheckIterError = new TaskCheckIterationError("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckIterError, "CheckIterError");

	TaskBase *tCheckAbsGap = new TaskCheckAbsoluteGap("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");

	TaskBase *tCheckRelGap = new TaskCheckRelativeGap("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");

	TaskBase *tCheckConstrTol = new TaskCheckConstraintTolerance("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckConstrTol, "CheckConstrTol");

	TaskBase *tSelectPrimSolPool = new TaskSelectPrimalCandidatesFromSolutionPool();
	ProcessInfo::getInstance().tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
	dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

	ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");
	ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");

	TaskBase *tCheckIterLim = new TaskCheckIterationLimit("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckIterLim, "CheckIterLim");

	TaskBase *tCheckTimeLim = new TaskCheckTimeLimit("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckTimeLim, "CheckTimeLim");

	//ProcessInfo::getInstance().tasks->addTask(tInitializeIteration, "InitIter");

	/*
	 // Needed because e.g. fac2 terminates with optimal linear solution but not optimal nonlinear solution
	 // TODO: figure out why and fix...
	 TaskBase *tForcedHyperplaneAddition = new TaskSequential();

	 TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsSolution();
	 dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tSelectHPPts);
	 dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tAddHPs);

	 TaskBase *tForceSupportingHyperplaneAddition = new TaskConditional();

	 dynamic_cast<TaskConditional*>(tForceSupportingHyperplaneAddition)->setCondition(
	 [this]()
	 {
	 auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	 if (prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal && prevIter->maxDeviation > Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	 {
	 ProcessInfo::getInstance().outputError(
	 "     Forced addition of cutting plane in solution point.");
	 return (true);
	 }

	 return (false);
	 });

	 dynamic_cast<TaskConditional*>(tForceSupportingHyperplaneAddition)->setTaskIfTrue(tForcedHyperplaneAddition);

	 ProcessInfo::getInstance().tasks->addTask(tForceSupportingHyperplaneAddition, "ForceSupportingHyperplaneAddition");
	 */
	TaskBase *tPrintBoundReport = new TaskPrintSolutionBoundReport();
	ProcessInfo::getInstance().tasks->addTask(tPrintBoundReport, "PrintBoundReport");

	//TaskBase *tGoto = new TaskGoto("PrintIterHeaderCheck");
	//ProcessInfo::getInstance().tasks->addTask(tGoto, "Goto");

	ProcessInfo::getInstance().tasks->addTask(tFinalizeSolution, "FinalizeSolution");

	TaskBase *tPrintSol = new TaskPrintSolution();
	ProcessInfo::getInstance().tasks->addTask(tPrintSol, "PrintSol");
}

SolutionStrategyLazy::~SolutionStrategyLazy()
{
// TODO Auto-generated destructor stub
}

bool SolutionStrategyLazy::solveProblem()
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

void SolutionStrategyLazy::initializeStrategy()
{

}
