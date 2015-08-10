/*
 * SolutionStrategySHOT.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: alundell
 */

#include <SolutionStrategySHOT.h>

SolutionStrategySHOT::SolutionStrategySHOT(OSInstance* osInstance)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->createTimer("Reformulation", "Time spent reformulating problem");

	processInfo->createTimer("InteriorPointTotal", "Time spent finding interior point");

	auto solver = static_cast<ES_NLPSolver>(settings->getIntSetting("NLPSolver", "NLP"));

	if (solver == ES_NLPSolver::CuttingPlaneMiniMax || solver == ES_NLPSolver::IPOptMiniMax)
	{
		processInfo->createTimer("InteriorPointMinimax", " - Solving minimax NLP problem");
	}
	else if (solver == ES_NLPSolver::IPOptRelaxed)
	{
		processInfo->createTimer("InteriorPointRelaxed", " - Solving relaxed NLP problem");
	}

	processInfo->createTimer("Subproblems", "Time spent solving subproblems");
	processInfo->createTimer("LP", " - LP problems");
	processInfo->createTimer("MILP", " - MILP problems");
	processInfo->createTimer("PopulateSolutionPool", " - Populate solution pool");
	processInfo->createTimer("HyperplaneLinesearch", " - Linesearch");
	processInfo->createTimer("PrimalBoundTotal", " - Primal solution search");
	processInfo->createTimer("PrimalBoundSearchNLP", "    - NLP");
	processInfo->createTimer("PrimalBoundLinesearch", "    - Linesearch");

	TaskBase *tFinalizeSolution = new TaskSequential();

	TaskBase *tInitOrigProblem = new TaskInitializeOriginalProblem(osInstance);
	processInfo->tasks->addTask(tInitOrigProblem, "InitOrigProb");

	TaskBase *tPrintProblemStats = new TaskPrintProblemStats();
	processInfo->tasks->addTask(tPrintProblemStats, "PrintProbStat");

	if (processInfo->originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic
			|| processInfo->originalProblem->getNumberOfNonlinearConstraints() != 0)
	{
		TaskBase *tFindIntPoint = new TaskFindInteriorPoint();
		processInfo->tasks->addTask(tFindIntPoint, "FindIntPoint");
	}

	TaskBase *tInitMILPSolver = new TaskInitializeMILPSolver();
	processInfo->tasks->addTask(tInitMILPSolver, "InitMILPSolver");

	TaskBase *tCreateMILPProblem = new TaskCreateMILPProblem();
	processInfo->tasks->addTask(tCreateMILPProblem, "CreateMILPProblem");

	TaskBase *tInitializeIteration = new TaskInitializeIteration();
	processInfo->tasks->addTask(tInitializeIteration, "InitIter");

	TaskBase *tExecuteRelaxStrategy = new TaskExecuteRelaxationStrategy();
	processInfo->tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategyInitial");

	TaskBase *tPrintIterHeaderCheck = new TaskConditional();
	TaskBase *tPrintIterHeader = new TaskPrintIterationHeader();

	dynamic_cast<TaskConditional*>(tPrintIterHeaderCheck)->setCondition([this]()
	{	return (processInfo->getCurrentIteration()->iterationNumber % 50 == 1);});
	dynamic_cast<TaskConditional*>(tPrintIterHeaderCheck)->setTaskIfTrue(tPrintIterHeader);

	processInfo->tasks->addTask(tPrintIterHeaderCheck, "PrintIterHeaderCheck");

	TaskBase *tSolveIteration = new TaskSolveIteration();
	processInfo->tasks->addTask(tSolveIteration, "SolveIter");

	/*if (processInfo->originalProblem->isObjectiveFunctionNonlinear())
	 {
	 TaskBase *tUpdateNonlinearObjectiveSolution = new TaskUpdateNonlinearObjectiveByLinesearch();
	 processInfo->tasks->addTask(tUpdateNonlinearObjectiveSolution, "UpdateNonlinearObjective");
	 }*/

	TaskBase *tPrintIterReport = new TaskPrintIterationReport();
	processInfo->tasks->addTask(tPrintIterReport, "PrintIterReport");

	TaskBase *tCheckIterError = new TaskCheckIterationError("FinalizeSolution");
	processInfo->tasks->addTask(tCheckIterError, "CheckIterError");

	TaskBase *tCheckConstrTol = new TaskCheckConstraintTolerance("FinalizeSolution");
	processInfo->tasks->addTask(tCheckConstrTol, "CheckConstrTol");

	TaskBase *tSelectPrimSolPool = new TaskSelectPrimalCandidatesFromSolutionPool();
	processInfo->tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
	dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

	TaskBase *tSelectPrimLinesearch = new TaskSelectPrimalCandidatesFromLinesearch();
	processInfo->tasks->addTask(tSelectPrimLinesearch, "SelectPrimLinesearch");
	dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimLinesearch);

	if (settings->getBoolSetting("UseNLPCall", "PrimalBound"))
	{
		TaskBase *tSelectPrimNLP = new TaskSelectPrimalCandidatesFromNLP();

		if (processInfo->originalProblem->getNumberOfNonlinearConstraints() > 0)
		{
			TaskBase *tSelectPrimNLPCheck = new TaskConditional();

			dynamic_cast<TaskConditional*>(tSelectPrimNLPCheck)->setCondition(
					[this]()
					{

						if (!processInfo->getCurrentIteration()->isMILP())
						{
							return (false);
						}

						if ( processInfo->itersMILPWithoutNLPCall >= settings->getIntSetting("NLPCallMaxIter", "PrimalBound"))
						{
							return (true);
						}

						if ( processInfo->itersWithStagnationMILP >= settings->getIntSetting("NLPCallMaxIter", "PrimalBound"))
						{
							return (true);
						}

						if (processInfo->getElapsedTime("Total") -processInfo->solTimeLastNLPCall > settings->getDoubleSetting("NLPCallMaxElapsedTime", "PrimalBound"))
						{
							return (true);
						}

						processInfo->itersMILPWithoutNLPCall++;

						return (false);
					});

			dynamic_cast<TaskConditional*>(tSelectPrimNLPCheck)->setTaskIfTrue(tSelectPrimNLP);

			processInfo->tasks->addTask(tSelectPrimNLPCheck, "SelectPrimNLPCheck");
			dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimNLP);
		}
	}

	TaskBase *tCheckPrimCands = new TaskCheckPrimalSolutionCandidates();
	processInfo->tasks->addTask(tCheckPrimCands, "CheckPrimCands");

	TaskBase *tCheckAbsGap = new TaskCheckAbsoluteGap("FinalizeSolution");
	processInfo->tasks->addTask(tCheckAbsGap, "CheckAbsGap");

	TaskBase *tCheckRelGap = new TaskCheckRelativeGap("FinalizeSolution");
	processInfo->tasks->addTask(tCheckRelGap, "CheckRelGap");

	TaskBase *tCheckObjStag = new TaskCheckObjectiveStagnation("FinalizeSolution");
	processInfo->tasks->addTask(tCheckObjStag, "CheckObjStag");

	TaskBase *tCheckIterLim = new TaskCheckIterationLimit("FinalizeSolution");
	processInfo->tasks->addTask(tCheckIterLim, "CheckIterLim");

	TaskBase *tCheckTimeLim = new TaskCheckTimeLimit("FinalizeSolution");
	processInfo->tasks->addTask(tCheckTimeLim, "CheckTimeLim");

	processInfo->tasks->addTask(tInitializeIteration, "InitIter");

	//TaskBase *tExecuteRelaxStrategy = new TaskExecuteRelaxationStrategy();
	processInfo->tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategy");

	TaskBase *tExecuteSolLimStrategy = new TaskExecuteSolutionLimitStrategy();
	processInfo->tasks->addTask(tExecuteSolLimStrategy, "ExecSolLimStrategy");

	TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsLinesearch();
	processInfo->tasks->addTask(tSelectHPPts, "SelectHPPts");

	processInfo->tasks->addTask(tCheckPrimCands, "CheckPrimCands2");
	processInfo->tasks->addTask(tCheckAbsGap, "CheckAbsGap");
	processInfo->tasks->addTask(tCheckRelGap, "CheckRelGap");

	TaskBase *tAddHPs = new TaskAddHyperplanes();
	processInfo->tasks->addTask(tAddHPs, "AddHPs");

	//TaskBase *tSolveFixedLP = new TaskSolveFixedLinearProblem();
	//processInfo->tasks->addTask(tSolveFixedLP, "SolveFixedLP");
	//processInfo->tasks->addTask(tCheckPrimCands, "CheckPrimCands2");
	//processInfo->tasks->addTask(tCheckAbsGap, "CheckAbsGap");
	//processInfo->tasks->addTask(tCheckRelGap, "CheckRelGap");

	TaskBase *tPrintBoundReport = new TaskPrintSolutionBoundReport();
	processInfo->tasks->addTask(tPrintBoundReport, "PrintBoundReport");

	TaskBase *tGoto = new TaskGoto("PrintIterHeaderCheck");
	processInfo->tasks->addTask(tGoto, "Goto");

	dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tCheckPrimCands);
	processInfo->tasks->addTask(tFinalizeSolution, "FinalizeSolution");

	TaskBase *tPrintSol = new TaskPrintSolution();
	processInfo->tasks->addTask(tPrintSol, "PrintSol");

	TaskBase *nextTask = new TaskBase;

	while (processInfo->tasks->getNextTask(nextTask))
	{
		//std::cout << "Next task is of type: " << nextTask->getType() << std::endl;
		nextTask->run();
		//std::cout << "Finished task" << std::endl;
	}

	processInfo->tasks->clearTasks();

}

SolutionStrategySHOT::~SolutionStrategySHOT()
{
// TODO Auto-generated destructor stub
}

bool SolutionStrategySHOT::solveProblem()
{
	return (true);
}

void SolutionStrategySHOT::initializeStrategy()
{

}
