
#include "SolutionStrategyMIQCQP.h"

SolutionStrategyMIQCQP::SolutionStrategyMIQCQP(OSInstance* osInstance)
{
	ProcessInfo::getInstance().createTimer("Reformulation", "Time spent reformulating problem");

	ProcessInfo::getInstance().createTimer("Subproblems", "Time spent solving subproblems");
	ProcessInfo::getInstance().createTimer("MIP", " - MIP problems");
	ProcessInfo::getInstance().createTimer("PrimalBoundTotal", " - Primal solution search");

	ProcessInfo::getInstance().createTimer("Reformulation", "Time spent reformulating problem");
	ProcessInfo::getInstance().createTimer("InteriorPointTotal", "Time spent finding interior point");

	auto solverMIP = static_cast<ES_MIPSolver>(Settings::getInstance().getIntSetting("MIP.Solver", "Dual"));

	TaskBase *tFinalizeSolution = new TaskSequential();

	TaskBase *tInitMIPSolver = new TaskInitializeDualSolver(solverMIP, false);
	ProcessInfo::getInstance().tasks->addTask(tInitMIPSolver, "InitMIPSolver");

	auto MIPSolver = ProcessInfo::getInstance().MIPSolver;

	TaskBase *tInitOrigProblem = new TaskInitializeOriginalProblem(osInstance);
	ProcessInfo::getInstance().tasks->addTask(tInitOrigProblem, "InitOrigProb");

	TaskBase *tPrintProblemStats = new TaskPrintProblemStats();
	ProcessInfo::getInstance().tasks->addTask(tPrintProblemStats, "PrintProbStat");

	TaskBase *tCreateDualProblem = new TaskCreateDualProblem(MIPSolver);
	ProcessInfo::getInstance().tasks->addTask(tCreateDualProblem, "CreateDualProblem");

	TaskBase *tInitializeIteration = new TaskInitializeIteration();
	ProcessInfo::getInstance().tasks->addTask(tInitializeIteration, "InitIter");

	TaskBase *tPrintIterHeader = new TaskPrintIterationHeader();

	ProcessInfo::getInstance().tasks->addTask(tPrintIterHeader, "PrintIterHeader");

	TaskBase *tSolveIteration = new TaskSolveIteration(MIPSolver);
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

	TaskBase *tPrintBoundReport = new TaskPrintSolutionBoundReport();
	ProcessInfo::getInstance().tasks->addTask(tPrintBoundReport, "PrintBoundReport");

	ProcessInfo::getInstance().tasks->addTask(tFinalizeSolution, "FinalizeSolution");

	TaskBase *tPrintSol = new TaskPrintSolution();
	ProcessInfo::getInstance().tasks->addTask(tPrintSol, "PrintSol");
}

SolutionStrategyMIQCQP::~SolutionStrategyMIQCQP()
{
// TODO Auto-generated destructor stub
}

bool SolutionStrategyMIQCQP::solveProblem()
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

void SolutionStrategyMIQCQP::initializeStrategy()
{

}
