/*
 * SolutionStrategySHOT.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: alundell
 */

#include <SolutionStrategyNLP.h>

SolutionStrategyNLP::SolutionStrategyNLP(OSInstance* osInstance)
{
	ProcessInfo::getInstance().createTimer("Reformulation", "Time spent reformulating problem");
	ProcessInfo::getInstance().createTimer("InteriorPointTotal", "Time spent finding interior point");

	auto solver = static_cast<ES_NLPSolver>(Settings::getInstance().getIntSetting("InteriorPointSolver",
			"InteriorPoint"));
	ProcessInfo::getInstance().createTimer("InteriorPoint", " - Solving interior point NLP problem");

	ProcessInfo::getInstance().createTimer("Subproblems", "Time spent solving subproblems");
	ProcessInfo::getInstance().createTimer("LP", " - Relaxed problems");
	ProcessInfo::getInstance().createTimer("MILP", " - MIP problems");
	ProcessInfo::getInstance().createTimer("HyperplaneLinesearch", " - Linesearch");
	ProcessInfo::getInstance().createTimer("ObjectiveLinesearch", " - Objective linesearch");
	ProcessInfo::getInstance().createTimer("PrimalBoundTotal", " - Primal solution search");
	ProcessInfo::getInstance().createTimer("PrimalBoundLinesearch", "    - Linesearch");

	auto solverMILP = static_cast<ES_MILPSolver>(Settings::getInstance().getIntSetting("MILPSolver", "MILP"));

	TaskBase *tFinalizeSolution = new TaskSequential();

	TaskBase *tInitMILPSolver = new TaskInitializeMILPSolver(solverMILP, false);
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

	TaskBase *tExecuteRelaxStrategy = new TaskExecuteRelaxationStrategy(MILPSolver);
	ProcessInfo::getInstance().tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategyInitial");

	/*if (ProcessInfo::getInstance().originalProblem->getNumberOfBinaryVariables()
	 + ProcessInfo::getInstance().originalProblem->getNumberOfIntegerVariables() > 0)
	 {
	 TaskBase *tExecuteRelaxStrategy = new TaskExecuteRelaxationStrategy(MILPSolver);
	 ProcessInfo::getInstance().tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategyInitial");
	 }*/

	TaskBase *tPrintIterHeaderCheck = new TaskConditional();
	TaskBase *tPrintIterHeader = new TaskPrintIterationHeader();

	dynamic_cast<TaskConditional*>(tPrintIterHeaderCheck)->setCondition([this]()
	{	return (ProcessInfo::getInstance().getCurrentIteration()->iterationNumber % 50 == 1);});
	dynamic_cast<TaskConditional*>(tPrintIterHeaderCheck)->setTaskIfTrue(tPrintIterHeader);

	ProcessInfo::getInstance().tasks->addTask(tPrintIterHeaderCheck, "PrintIterHeaderCheck");

	if (static_cast<ES_PresolveStrategy>(Settings::getInstance().getIntSetting("PresolveStrategy", "Presolve"))
			!= ES_PresolveStrategy::Never)
	{
		TaskBase *tPresolve = new TaskPresolve(MILPSolver);
		ProcessInfo::getInstance().tasks->addTask(tPresolve, "Presolve");
	}

	TaskBase *tSolveIteration = new TaskSolveIteration(MILPSolver);
	ProcessInfo::getInstance().tasks->addTask(tSolveIteration, "SolveIter");

	if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear()
			&& Settings::getInstance().getBoolSetting("UseObjectiveLinesearch", "PrimalBound"))
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
	dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

	if (Settings::getInstance().getBoolSetting("PrimalStrategyLinesearch", "PrimalBound"))
	{
		TaskBase *tSelectPrimLinesearch = new TaskSelectPrimalCandidatesFromLinesearch();
		ProcessInfo::getInstance().tasks->addTask(tSelectPrimLinesearch, "SelectPrimLinesearch");
		dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimLinesearch);

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

	ProcessInfo::getInstance().tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategy");

	if (static_cast<ES_HyperplanePointStrategy>(Settings::getInstance().getIntSetting("HyperplanePointStrategy",
			"Algorithm")) == ES_HyperplanePointStrategy::ESH)
	{
		if (static_cast<ES_LinesearchConstraintStrategy>(Settings::getInstance().getIntSetting(
				"LinesearchConstraintStrategy", "ESH")) == ES_LinesearchConstraintStrategy::AllAsMaxFunct)
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
