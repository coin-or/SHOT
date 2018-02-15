/*
 * SolutionStrategySHOT.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: alundell
 */

#include "SolutionStrategyNLP.h"

SolutionStrategyNLP::SolutionStrategyNLP(OSInstance *osInstance)
{
	ProcessInfo::getInstance().createTimer("Reformulation", "Time spent reformulating problem");
	ProcessInfo::getInstance().createTimer("InteriorPointTotal", "Time spent finding interior point");

	auto solver = static_cast<ES_NLPSolver>(Settings::getInstance().getIntSetting("ESH.InteriorPoint.Solver", "Dual"));
	ProcessInfo::getInstance().createTimer("InteriorPoint", " - Solving interior point NLP problem");

	ProcessInfo::getInstance().createTimer("Subproblems", "Time spent solving subproblems");
	ProcessInfo::getInstance().createTimer("LP", " - Relaxed problems");
	ProcessInfo::getInstance().createTimer("MIP", " - MIP problems");
	ProcessInfo::getInstance().createTimer("HyperplaneLinesearch", " - Linesearch");
	ProcessInfo::getInstance().createTimer("ObjectiveLinesearch", " - Objective linesearch");
	ProcessInfo::getInstance().createTimer("PrimalBoundTotal", " - Primal solution search");
	ProcessInfo::getInstance().createTimer("PrimalBoundLinesearch", "    - Linesearch");

	auto solverMIP = static_cast<ES_MIPSolver>(Settings::getInstance().getIntSetting("MIP.Solver", "Dual"));

	TaskBase *tFinalizeSolution = new TaskSequential();

	TaskBase *tInitMIPSolver = new TaskInitializeDualSolver(solverMIP, false);
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

	//TaskBase *tExecuteRelaxStrategy = new TaskExecuteRelaxationStrategy(MIPSolver);
	//ProcessInfo::getInstance().tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategyInitial");

	/*if (ProcessInfo::getInstance().originalProblem->getNumberOfBinaryVariables()
	 + ProcessInfo::getInstance().originalProblem->getNumberOfIntegerVariables() > 0)
	 {
	 TaskBase *tExecuteRelaxStrategy = new TaskExecuteRelaxationStrategy(MIPSolver);
	 ProcessInfo::getInstance().tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategyInitial");
	 }*/

	TaskBase *tPrintIterHeaderCheck = new TaskConditional();
	TaskBase *tPrintIterHeader = new TaskPrintIterationHeader();

	dynamic_cast<TaskConditional *>(tPrintIterHeaderCheck)->setCondition([this]() { return (ProcessInfo::getInstance().getCurrentIteration()->iterationNumber % 50 == 1); });
	dynamic_cast<TaskConditional *>(tPrintIterHeaderCheck)->setTaskIfTrue(tPrintIterHeader);

	ProcessInfo::getInstance().tasks->addTask(tPrintIterHeaderCheck, "PrintIterHeaderCheck");

	if (static_cast<ES_MIPPresolveStrategy>(Settings::getInstance().getIntSetting("MIP.Presolve.Frequency", "Dual")) != ES_MIPPresolveStrategy::Never)
	{
		TaskBase *tPresolve = new TaskPresolve(MIPSolver);
		ProcessInfo::getInstance().tasks->addTask(tPresolve, "Presolve");
	}

	TaskBase *tSolveIteration = new TaskSolveIteration(MIPSolver);
	ProcessInfo::getInstance().tasks->addTask(tSolveIteration, "SolveIter");

	if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear() && Settings::getInstance().getBoolSetting("ObjectiveLinesearch.Use", "Dual"))
	{
		TaskBase *tUpdateNonlinearObjectiveSolution = new TaskUpdateNonlinearObjectiveByLinesearch();
		ProcessInfo::getInstance().tasks->addTask(tUpdateNonlinearObjectiveSolution, "UpdateNonlinearObjective");
	}

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
	dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

	if (Settings::getInstance().getBoolSetting("Linesearch.Use", "Primal"))
	{
		TaskBase *tSelectPrimLinesearch = new TaskSelectPrimalCandidatesFromLinesearch();
		ProcessInfo::getInstance().tasks->addTask(tSelectPrimLinesearch, "SelectPrimLinesearch");
		dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimLinesearch);

		ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");

		ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");
	}

	TaskBase *tCheckObjStag = new TaskCheckObjectiveStagnation("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckObjStag, "CheckObjStag");

	TaskBase *tCheckIterLim = new TaskCheckIterationLimit("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckIterLim, "CheckIterLim");

	TaskBase *tCheckTimeLim = new TaskCheckTimeLimit("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckTimeLim, "CheckTimeLim");

	ProcessInfo::getInstance().tasks->addTask(tInitializeIteration, "InitIter");

	//ProcessInfo::getInstance().tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategy");

	if (static_cast<ES_HyperplaneCutStrategy>(Settings::getInstance().getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
	{
		TaskBase *tUpdateInteriorPoint = new TaskUpdateInteriorPoint();
		ProcessInfo::getInstance().tasks->addTask(tUpdateInteriorPoint, "UpdateInteriorPoint");

		if (static_cast<ES_RootsearchConstraintStrategy>(Settings::getInstance().getIntSetting(
				"ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
		{
			TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsLinesearch();
			ProcessInfo::getInstance().tasks->addTask(tSelectHPPts, "SelectHPPts");
		}
		else
		{
			TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsIndividualLinesearch();
			ProcessInfo::getInstance().tasks->addTask(tSelectHPPts, "SelectHPPts");
		}
	}
	else
	{
		TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsSolution();
		ProcessInfo::getInstance().tasks->addTask(tSelectHPPts, "SelectHPPts");
	}

	//ProcessInfo::getInstance().tasks->addTask(tCheckPrimCands, "CheckPrimCands");
	ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");
	ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");

	//TaskBase *tAddHPs = new TaskAddHyperplanes();
	ProcessInfo::getInstance().tasks->addTask(tAddHPs, "AddHPs");

	TaskBase *tPrintBoundReport = new TaskPrintSolutionBoundReport();
	ProcessInfo::getInstance().tasks->addTask(tPrintBoundReport, "PrintBoundReport");

	TaskBase *tGoto = new TaskGoto("PrintIterHeaderCheck");
	ProcessInfo::getInstance().tasks->addTask(tGoto, "Goto");

	//dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tCheckPrimCands);
	//dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tCheckDualCands);

	ProcessInfo::getInstance().tasks->addTask(tFinalizeSolution, "FinalizeSolution");

	TaskBase *tPrintSol = new TaskPrintSolution();
	ProcessInfo::getInstance().tasks->addTask(tPrintSol, "PrintSol");
}

SolutionStrategyNLP::~SolutionStrategyNLP()
{
	// TODO Auto-generated destructor stub
}

bool SolutionStrategyNLP::solveProblem()
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

void SolutionStrategyNLP::initializeStrategy()
{
}
