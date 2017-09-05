/*
 * SolutionStrategySHOT.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: alundell
 */

#include "SolutionStrategySHOT.h"

SolutionStrategySHOT::SolutionStrategySHOT(OSInstance* osInstance)
{
	//processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	ProcessInfo::getInstance().createTimer("Reformulation", "Time spent reformulating problem");

	ProcessInfo::getInstance().createTimer("InteriorPointTotal", "Time spent finding interior point");

	auto solver = static_cast<ES_NLPSolver>(settings->getIntSetting("InteriorPointSolver", "InteriorPoint"));
	ProcessInfo::getInstance().createTimer("InteriorPoint", " - Solving interior point NLP problem");

	ProcessInfo::getInstance().createTimer("Subproblems", "Time spent solving subproblems");
	ProcessInfo::getInstance().createTimer("LP", " - Relaxed problems");
	ProcessInfo::getInstance().createTimer("MILP", " - MIP problems");
	ProcessInfo::getInstance().createTimer("PopulateSolutionPool", " - Populate solution pool");
	ProcessInfo::getInstance().createTimer("LazyChange", " - Change to lazy constraints");
	ProcessInfo::getInstance().createTimer("HyperplaneLinesearch", " - Linesearch");
	ProcessInfo::getInstance().createTimer("ObjectiveLinesearch", " - Objective linesearch");
	ProcessInfo::getInstance().createTimer("PrimalBoundTotal", " - Primal solution search");
	ProcessInfo::getInstance().createTimer("PrimalBoundSearchNLP", "    - NLP");
	ProcessInfo::getInstance().createTimer("PrimalBoundLinesearch", "    - Linesearch");
	ProcessInfo::getInstance().createTimer("PrimalBoundFixedLP", "    - Fixed LP");

	auto solverMILP = static_cast<ES_MILPSolver>(settings->getIntSetting("MILPSolver", "MILP"));

	TaskBase *tFinalizeSolution = new TaskSequential();

	TaskBase *tInitMILPSolver = new TaskInitializeMILPSolver(osInstance);
	ProcessInfo::getInstance().tasks->addTask(tInitMILPSolver, "InitMILPSolver");

	auto MILPSolver = ProcessInfo::getInstance().MILPSolver;

	TaskBase *tInitOrigProblem = new TaskInitializeOriginalProblem(osInstance);
	ProcessInfo::getInstance().tasks->addTask(tInitOrigProblem, "InitOrigProb");

	TaskBase *tPrintProblemStats = new TaskPrintProblemStats();
	ProcessInfo::getInstance().tasks->addTask(tPrintProblemStats, "PrintProbStat");

	if (ProcessInfo::getInstance().originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic
			|| ProcessInfo::getInstance().originalProblem->getNumberOfNonlinearConstraints() != 0)
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

	TaskBase *tPrintIterHeaderCheck = new TaskConditional();
	TaskBase *tPrintIterHeader = new TaskPrintIterationHeader();

	dynamic_cast<TaskConditional*>(tPrintIterHeaderCheck)->setCondition([this]()
	{	return (ProcessInfo::getInstance().getCurrentIteration()->iterationNumber % 50 == 1);});
	dynamic_cast<TaskConditional*>(tPrintIterHeaderCheck)->setTaskIfTrue(tPrintIterHeader);

	ProcessInfo::getInstance().tasks->addTask(tPrintIterHeaderCheck, "PrintIterHeaderCheck");

	if (static_cast<ES_PresolveStrategy>(settings->getIntSetting("PresolveStrategy", "Presolve"))
			!= ES_PresolveStrategy::Never)
	{
		TaskBase *tPresolve = new TaskPresolve(MILPSolver);
		ProcessInfo::getInstance().tasks->addTask(tPresolve, "Presolve");
	}

	TaskBase *tSolveIteration = new TaskSolveIteration(MILPSolver);
	ProcessInfo::getInstance().tasks->addTask(tSolveIteration, "SolveIter");

	if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear()
			&& settings->getBoolSetting("UseObjectiveLinesearch", "PrimalBound")
			&& solverMILP != ES_MILPSolver::CplexExperimental)
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

	if (static_cast<ES_SolutionStrategy>(settings->getIntSetting("SolutionStrategy", "Algorithm"))
			== ES_SolutionStrategy::ESH)
	{
		TaskBase *tSelectPrimLinesearch = new TaskSelectPrimalCandidatesFromLinesearch();
		ProcessInfo::getInstance().tasks->addTask(tSelectPrimLinesearch, "SelectPrimLinesearch");
		dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimLinesearch);

		ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");

		ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");
	}

	if (settings->getBoolSetting("SolveFixedLP", "Algorithm"))
	{
		TaskBase *tSolveFixedLP = new TaskSolveFixedLinearProblem(MILPSolver);
		ProcessInfo::getInstance().tasks->addTask(tSolveFixedLP, "SolveFixedLP");
		ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");
		ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");
	}

	if (settings->getIntSetting("NLPFixedStrategy", "PrimalBound") != static_cast<int>(ES_PrimalNLPStrategy::DoNotUse)
			&& ProcessInfo::getInstance().originalProblem->getNumberOfNonlinearConstraints() > 0)
	{
		TaskBase *tSelectPrimFixedNLPSolPool = new TaskSelectPrimalFixedNLPPointsFromSolutionPool();
		ProcessInfo::getInstance().tasks->addTask(tSelectPrimFixedNLPSolPool, "SelectPrimFixedNLPSolPool");
		dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimFixedNLPSolPool);

		TaskBase *tSelectPrimNLPCheck = new TaskSelectPrimalCandidatesFromNLP();
		ProcessInfo::getInstance().tasks->addTask(tSelectPrimNLPCheck, "SelectPrimNLPCheck");
		dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimNLPCheck);
	}

	/*
	 if (settings->getBoolSetting("UseNLPCall", "PrimalBound"))
	 {
	 TaskBase *tSelectPrimNLP = new TaskSelectPrimalCandidatesFromNLP();

	 if (ProcessInfo::getInstance().originalProblem->getNumberOfNonlinearConstraints() > 0)
	 {
	 TaskBase *tSelectPrimNLPCheck = new TaskConditional();

	 dynamic_cast<TaskConditional*>(tSelectPrimNLPCheck)->setCondition([this]()
	 {
	 auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	 // Added MILPSollimit updated krav mars 2016
	 if (!currIter->isMILP() || currIter->solutionPoints.size() == 0 || currIter->MILPSolutionLimitUpdated)
	 {
	 return (false);
	 }

	 if ( ProcessInfo::getInstance().itersMILPWithoutNLPCall >= settings->getIntSetting("NLPFixedMaxElapsedTime", "PrimalBound"))
	 {
	 return (true);
	 }

	 //if ( ProcessInfo::getInstance().itersWithStagnationMILP >= settings->getIntSetting("NLPFixedMaxElapsedTime", "PrimalBound"))
	 //{
	 //return (true);
	 //}

	 if (ProcessInfo::getInstance().getElapsedTime("Total") -ProcessInfo::getInstance().solTimeLastNLPCall > settings->getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound"))
	 {
	 return (true);
	 }

	 int maxItersNoMIPChange = 20;
	 auto currSolPt = currIter->solutionPoints.at(0).point;

	 bool noMIPChange = true;

	 for (int i = 1; i < maxItersNoMIPChange; i++)
	 {
	 if (ProcessInfo::getInstance().iterations.size() <= i)
	 {
	 noMIPChange = false;
	 break;
	 }

	 auto prevIter = &ProcessInfo::getInstance().iterations.at(currIter->iterationNumber -1 - i);

	 if (!prevIter->isMILP())
	 {
	 noMIPChange = false;
	 break;
	 }

	 auto discreteIdxs = ProcessInfo::getInstance().originalProblem->getDiscreteVariableIndices();

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
	 ProcessInfo::getInstance().outputWarning("     MIP solution has not changed in " + to_string( maxItersNoMIPChange)+ " iterations. Solving NLP problem...");
	 return (true);
	 }

	 ProcessInfo::getInstance().itersMILPWithoutNLPCall++;

	 return (false);
	 });

	 dynamic_cast<TaskConditional*>(tSelectPrimNLPCheck)->setTaskIfTrue(tSelectPrimNLP);

	 ProcessInfo::getInstance().tasks->addTask(tSelectPrimNLPCheck, "SelectPrimNLPCheck");
	 dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimNLP);

	 ProcessInfo::getInstance().tasks->addTask(tCheckPrimCands, "CheckPrimCands");
	 ProcessInfo::getInstance().tasks->addTask(tCheckDualCands, "CheckDualCands");
	 ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");
	 ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");
	 }
	 }*/

	TaskBase *tCheckObjStag = new TaskCheckObjectiveStagnation("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckObjStag, "CheckObjStag");

	TaskBase *tCheckIterLim = new TaskCheckIterationLimit("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckIterLim, "CheckIterLim");

	TaskBase *tCheckTimeLim = new TaskCheckTimeLimit("FinalizeSolution");
	ProcessInfo::getInstance().tasks->addTask(tCheckTimeLim, "CheckTimeLim");

	ProcessInfo::getInstance().tasks->addTask(tInitializeIteration, "InitIter");

	//TaskBase *tExecuteRelaxStrategy = new TaskExecuteRelaxationStrategy();
	ProcessInfo::getInstance().tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategy");

	TaskBase *tExecuteSolLimStrategy = new TaskExecuteSolutionLimitStrategy(MILPSolver);
	ProcessInfo::getInstance().tasks->addTask(tExecuteSolLimStrategy, "ExecSolLimStrategy");

	if (solverMILP != ES_MILPSolver::CplexExperimental)
	{

		if (static_cast<ES_SolutionStrategy>(settings->getIntSetting("SolutionStrategy", "Algorithm"))
				== ES_SolutionStrategy::ESH)
		{
			if (static_cast<ES_LinesearchConstraintStrategy>(settings->getIntSetting("LinesearchConstraintStrategy",
					"ESH")) == ES_LinesearchConstraintStrategy::AllAsMaxFunct)
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

		if (settings->getBoolSetting("AddIntegerCuts", "Algorithm"))
		{
			TaskBase *tAddICs = new TaskAddIntegerCuts(MILPSolver);
			ProcessInfo::getInstance().tasks->addTask(tAddICs, "AddICs");
		}

		/*
		 if (settings->getBoolSetting("UseLazyConstraints", "MILP"))
		 {
		 TaskBase *tSwitchLazy = new TaskSwitchToLazyConstraints();
		 ProcessInfo::getInstance().tasks->addTask(tSwitchLazy, "SwitchLazy");
		 }*/
	}
	else
	{
		// Needed because e.g. fac2 terminates with optimal linear solution but not optimal nonlinear solution
		TaskBase *tForcedHyperplaneAddition = new TaskSequential();

		TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsLinesearch();
		dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tSelectHPPts);
		//dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tCheckPrimCands);
		dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tCheckAbsGap);
		dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tCheckRelGap);

		//TaskBase *tAddHPs = new TaskAddHyperplanes();
		dynamic_cast<TaskSequential*>(tForcedHyperplaneAddition)->addTask(tAddHPs);

		TaskBase *tForceSupportingHyperplaneAddition = new TaskConditional();

		dynamic_cast<TaskConditional*>(tForceSupportingHyperplaneAddition)->setCondition(
				[this]()
				{
					auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

					if (prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal && prevIter->maxDeviation > settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
					{
						return (true);
					}

					return (false);
				});

		dynamic_cast<TaskConditional*>(tForceSupportingHyperplaneAddition)->setTaskIfTrue(tForcedHyperplaneAddition);

		ProcessInfo::getInstance().tasks->addTask(tForceSupportingHyperplaneAddition, "ForceSupportingHyperplaneAddition");
	}

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

SolutionStrategySHOT::~SolutionStrategySHOT()
{
// TODO Auto-generated destructor stub
}

bool SolutionStrategySHOT::solveProblem()
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

void SolutionStrategySHOT::initializeStrategy()
{

}
