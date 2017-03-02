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
	processInfo->createTimer("LP", " - Relaxed problems");
	processInfo->createTimer("MILP", " - MIP problems");
	processInfo->createTimer("PopulateSolutionPool", " - Populate solution pool");
	processInfo->createTimer("LazyChange", " - Change to lazy constraints");
	processInfo->createTimer("HyperplaneLinesearch", " - Linesearch");
	processInfo->createTimer("ObjectiveLinesearch", " - Objective linesearch");
	processInfo->createTimer("PrimalBoundTotal", " - Primal solution search");
	processInfo->createTimer("PrimalBoundSearchNLP", "    - NLP");
	processInfo->createTimer("PrimalBoundLinesearch", "    - Linesearch");
	processInfo->createTimer("PrimalBoundFixedLP", "    - Fixed LP");

	auto solverMILP = static_cast<ES_MILPSolver>(settings->getIntSetting("MILPSolver", "MILP"));

	TaskBase *tFinalizeSolution = new TaskSequential();

	TaskBase *tInitMILPSolver = new TaskInitializeMILPSolver(osInstance);
	processInfo->tasks->addTask(tInitMILPSolver, "InitMILPSolver");

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

	if (processInfo->originalProblem->isObjectiveFunctionNonlinear()
			&& settings->getBoolSetting("UseObjectiveLinesearch", "PrimalBound")
			&& solverMILP != ES_MILPSolver::CplexExperimental)
	{
		TaskBase *tUpdateNonlinearObjectiveSolution = new TaskUpdateNonlinearObjectiveByLinesearch();
		processInfo->tasks->addTask(tUpdateNonlinearObjectiveSolution, "UpdateNonlinearObjective");
	}

	TaskBase *tPrintIterReport = new TaskPrintIterationReport();
	processInfo->tasks->addTask(tPrintIterReport, "PrintIterReport");

	TaskBase *tCheckIterError = new TaskCheckIterationError("FinalizeSolution");
	processInfo->tasks->addTask(tCheckIterError, "CheckIterError");

	TaskBase *tSelectPrimSolPool = new TaskSelectPrimalCandidatesFromSolutionPool();
	processInfo->tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
	dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

	TaskBase *tCheckPrimCands = new TaskCheckPrimalSolutionCandidates();

	if (static_cast<ES_SolutionStrategy>(settings->getIntSetting("SolutionStrategy", "Algorithm"))
			== ES_SolutionStrategy::ESH)
	{
		TaskBase *tSelectPrimLinesearch = new TaskSelectPrimalCandidatesFromLinesearch();
		processInfo->tasks->addTask(tSelectPrimLinesearch, "SelectPrimLinesearch");
		dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimLinesearch);

		processInfo->tasks->addTask(tCheckPrimCands, "CheckPrimCands");
	}

	TaskBase *tCheckDualCands = new TaskCheckDualSolutionCandidates();
	processInfo->tasks->addTask(tCheckDualCands, "CheckDualCands");

	TaskBase *tCheckAbsGap = new TaskCheckAbsoluteGap("FinalizeSolution");
	processInfo->tasks->addTask(tCheckAbsGap, "CheckAbsGap");

	TaskBase *tCheckRelGap = new TaskCheckRelativeGap("FinalizeSolution");
	processInfo->tasks->addTask(tCheckRelGap, "CheckRelGap");

	TaskBase *tCheckConstrTol = new TaskCheckConstraintTolerance("FinalizeSolution");
	processInfo->tasks->addTask(tCheckConstrTol, "CheckConstrTol");

	if (settings->getBoolSetting("SolveFixedLP", "Algorithm"))
	{
		TaskBase *tSolveFixedLP = new TaskSolveFixedLinearProblem();
		processInfo->tasks->addTask(tSolveFixedLP, "SolveFixedLP");
		processInfo->tasks->addTask(tCheckPrimCands, "CheckPrimCands2");
		processInfo->tasks->addTask(tCheckDualCands, "CheckDualCands2");
		processInfo->tasks->addTask(tCheckAbsGap, "CheckAbsGap");
		processInfo->tasks->addTask(tCheckRelGap, "CheckRelGap");
	}

	if (settings->getBoolSetting("UseNLPCall", "PrimalBound"))
	{
		TaskBase *tSelectPrimNLP = new TaskSelectPrimalCandidatesFromNLP();

		if (processInfo->originalProblem->getNumberOfNonlinearConstraints() > 0)
		{
			TaskBase *tSelectPrimNLPCheck = new TaskConditional();

			dynamic_cast<TaskConditional*>(tSelectPrimNLPCheck)->setCondition([this]()
			{
				auto currIter = processInfo->getCurrentIteration();

				// Added MILPSollimit updated krav mars 2016
					if (!currIter->isMILP() || currIter->solutionPoints.size() == 0 || currIter->MILPSolutionLimitUpdated)
					{
						return (false);
					}

					if ( processInfo->itersMILPWithoutNLPCall >= settings->getIntSetting("NLPCallMaxIter", "PrimalBound"))
					{
						return (true);
					}

					/*if ( processInfo->itersWithStagnationMILP >= settings->getIntSetting("NLPCallMaxIter", "PrimalBound"))
					 {
					 return (true);
					 }*/

					if (processInfo->getElapsedTime("Total") -processInfo->solTimeLastNLPCall > settings->getDoubleSetting("NLPCallMaxElapsedTime", "PrimalBound"))
					{
						return (true);
					}

					int maxItersNoMIPChange = 20;
					auto currSolPt = currIter->solutionPoints.at(0).point;

					bool noMIPChange = true;

					for (int i = 1; i < maxItersNoMIPChange; i++)
					{
						if (processInfo->iterations.size() <= i)
						{
							noMIPChange = false;
							break;
						}

						auto prevIter = &processInfo->iterations.at(currIter->iterationNumber -1 - i);

						if (!prevIter->isMILP())
						{
							noMIPChange = false;
							break;
						}

						auto discreteIdxs = processInfo->originalProblem->getDiscreteVariableIndices();

						bool isDifferent = UtilityFunctions::isDifferentSelectedElements(currSolPt, prevIter->solutionPoints.at(0).point,
								discreteIdxs);

						if (isDifferent)
						{
							noMIPChange = false;
							break;
						}
					}

					if (noMIPChange)
					{
						processInfo->outputWarning("     MIP solution has not changed in " + to_string( maxItersNoMIPChange)+ " iterations. Solving NLP problem...");
						return (true);
					}

					processInfo->itersMILPWithoutNLPCall++;

					return (false);
				});

			dynamic_cast<TaskConditional*>(tSelectPrimNLPCheck)->setTaskIfTrue(tSelectPrimNLP);

			processInfo->tasks->addTask(tSelectPrimNLPCheck, "SelectPrimNLPCheck");
			dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimNLP);

			processInfo->tasks->addTask(tCheckPrimCands, "CheckPrimCands");
			processInfo->tasks->addTask(tCheckDualCands, "CheckDualCands");
			processInfo->tasks->addTask(tCheckAbsGap, "CheckAbsGap");
			processInfo->tasks->addTask(tCheckRelGap, "CheckRelGap");
		}
	}

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

	if (solverMILP != ES_MILPSolver::CplexExperimental)
	{

		if (static_cast<ES_SolutionStrategy>(settings->getIntSetting("SolutionStrategy", "Algorithm"))
				== ES_SolutionStrategy::ESH)
		{
			TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsIndividualLinesearch();
			processInfo->tasks->addTask(tSelectHPPts, "SelectHPPts");
		}
		else
		{
			TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsSolution();
			processInfo->tasks->addTask(tSelectHPPts, "SelectHPPts");
		}

		processInfo->tasks->addTask(tCheckPrimCands, "CheckPrimCands");
		processInfo->tasks->addTask(tCheckAbsGap, "CheckAbsGap");
		processInfo->tasks->addTask(tCheckRelGap, "CheckRelGap");

		TaskBase *tAddHPs = new TaskAddHyperplanes();
		processInfo->tasks->addTask(tAddHPs, "AddHPs");

		/*
		 if (settings->getBoolSetting("UseLazyConstraints", "MILP"))
		 {
		 TaskBase *tSwitchLazy = new TaskSwitchToLazyConstraints();
		 processInfo->tasks->addTask(tSwitchLazy, "SwitchLazy");
		 }*/
	}
	else
	{
		// Needed because e.g. fac2 terminates with optimal linear solution but not optimal nonlinear solution
		TaskBase *tForcedHyperplaneAddition = new TaskSequential();

		TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsLinesearch();
		dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tSelectHPPts);
		dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tCheckPrimCands);
		dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tCheckAbsGap);
		dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tCheckRelGap);

		TaskBase *tAddHPs = new TaskAddHyperplanes();
		dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tAddHPs);

		TaskBase *tForceSupportingHyperplaneAddition = new TaskConditional();

		dynamic_cast<TaskConditional*>(tForceSupportingHyperplaneAddition)->setCondition(
				[this]()
				{
					auto prevIter = processInfo->getPreviousIteration();

					if (prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal && prevIter->maxDeviation > settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
					{
						return (true);
					}

					return (false);
				});

		dynamic_cast<TaskConditional*>(tForceSupportingHyperplaneAddition)->setTaskIfTrue(tForcedHyperplaneAddition);

		processInfo->tasks->addTask(tForceSupportingHyperplaneAddition, "ForceSupportingHyperplaneAddition");
	}

	TaskBase *tPrintBoundReport = new TaskPrintSolutionBoundReport();
	processInfo->tasks->addTask(tPrintBoundReport, "PrintBoundReport");

	TaskBase *tGoto = new TaskGoto("PrintIterHeaderCheck");
	processInfo->tasks->addTask(tGoto, "Goto");

	dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tCheckPrimCands);
	dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tCheckDualCands);

	processInfo->tasks->addTask(tFinalizeSolution, "FinalizeSolution");

	TaskBase *tPrintSol = new TaskPrintSolution();
	processInfo->tasks->addTask(tPrintSol, "PrintSol");

}

SolutionStrategySHOT::~SolutionStrategySHOT()
{
// TODO Auto-generated destructor stub
}

bool SolutionStrategySHOT::solveProblem()
{
	TaskBase *nextTask = new TaskBase;

	while (processInfo->tasks->getNextTask(nextTask))
	{
		nextTask->run();
	}

	processInfo->tasks->clearTasks();

	return (true);
}

void SolutionStrategySHOT::initializeStrategy()
{

}
