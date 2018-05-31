/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "SolutionStrategyMultiTree.h"

SolutionStrategyMultiTree::SolutionStrategyMultiTree(OSInstance *osInstance)
{
    ProcessInfo::getInstance().createTimer("ProblemInitialization", " - problem initialization");
    ProcessInfo::getInstance().createTimer("InteriorPointSearch", " - interior point search");

    ProcessInfo::getInstance().createTimer("DualStrategy", " - dual strategy");
    ProcessInfo::getInstance().createTimer("DualProblemsRelaxed", "   - solving relaxed problems");
    ProcessInfo::getInstance().createTimer("DualProblemsIntegerFixed", "   - solving integer-fixed problems");
    ProcessInfo::getInstance().createTimer("DualProblemsDiscrete", "   - solving MIP problems");
    ProcessInfo::getInstance().createTimer("DualCutGenerationRootSearch", "   - performing root search for cuts");
    ProcessInfo::getInstance().createTimer("DualObjectiveLiftRootSearch", "   - performing root search for objective lift");

    ProcessInfo::getInstance().createTimer("PrimalStrategy", " - primal strategy");
    ProcessInfo::getInstance().createTimer("PrimalBoundStrategyNLP", "   - solving NLP problems");
    ProcessInfo::getInstance().createTimer("PrimalBoundStrategyRootSearch", "   - performing root searches");

    auto solver = static_cast<ES_InteriorPointStrategy>(Settings::getInstance().getIntSetting("ESH.InteriorPoint.Solver", "Dual"));
    auto solverMIP = static_cast<ES_MIPSolver>(Settings::getInstance().getIntSetting("MIP.Solver", "Dual"));

    TaskBase *tFinalizeSolution = new TaskSequential();

    TaskBase *tInitMIPSolver = new TaskInitializeDualSolver(solverMIP, false);
    ProcessInfo::getInstance().tasks->addTask(tInitMIPSolver, "InitMIPSolver");

    auto MIPSolver = ProcessInfo::getInstance().MIPSolver;

    TaskBase *tInitOrigProblem = new TaskInitializeOriginalProblem(osInstance);
    ProcessInfo::getInstance().tasks->addTask(tInitOrigProblem, "InitOrigProb");

    if (Settings::getInstance().getIntSetting("CutStrategy", "Dual") == (int)ES_HyperplaneCutStrategy::ESH && (ProcessInfo::getInstance().originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic || ProcessInfo::getInstance().originalProblem->getNumberOfNonlinearConstraints() != 0))
    {
        TaskBase *tFindIntPoint = new TaskFindInteriorPoint();
        ProcessInfo::getInstance().tasks->addTask(tFindIntPoint, "FindIntPoint");
    }

    TaskBase *tCreateDualProblem = new TaskCreateDualProblem(MIPSolver);
    ProcessInfo::getInstance().tasks->addTask(tCreateDualProblem, "CreateMILPProblem");

    TaskBase *tInitializeLinesearch = new TaskInitializeLinesearch();
    ProcessInfo::getInstance().tasks->addTask(tInitializeLinesearch, "InitializeLinesearch");

    TaskBase *tInitializeIteration = new TaskInitializeIteration();
    ProcessInfo::getInstance().tasks->addTask(tInitializeIteration, "InitIter");

    TaskBase *tAddHPs = new TaskAddHyperplanes(MIPSolver);
    ProcessInfo::getInstance().tasks->addTask(tAddHPs, "AddHPs");

    TaskBase *tExecuteRelaxStrategy = new TaskExecuteRelaxationStrategy(MIPSolver);
    ProcessInfo::getInstance().tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategyInitial");

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

    TaskBase *tSelectPrimSolPool = new TaskSelectPrimalCandidatesFromSolutionPool();
    ProcessInfo::getInstance().tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
    dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

    if (Settings::getInstance().getBoolSetting("Linesearch.Use", "Primal"))
    {
        TaskBase *tSelectPrimLinesearch = new TaskSelectPrimalCandidatesFromLinesearch();
        ProcessInfo::getInstance().tasks->addTask(tSelectPrimLinesearch, "SelectPrimLinesearch");
        dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimLinesearch);
    }

    TaskBase *tPrintIterReport = new TaskPrintIterationReport();
    ProcessInfo::getInstance().tasks->addTask(tPrintIterReport, "PrintIterReport");

    TaskBase *tCheckAbsGap = new TaskCheckAbsoluteGap("FinalizeSolution");
    ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");

    TaskBase *tCheckRelGap = new TaskCheckRelativeGap("FinalizeSolution");
    ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");

    TaskBase *tCheckIterLim = new TaskCheckIterationLimit("FinalizeSolution");
    ProcessInfo::getInstance().tasks->addTask(tCheckIterLim, "CheckIterLim");

    TaskBase *tCheckTimeLim = new TaskCheckTimeLimit("FinalizeSolution");
    ProcessInfo::getInstance().tasks->addTask(tCheckTimeLim, "CheckTimeLim");

    TaskBase *tCheckConstrTol = new TaskCheckConstraintTolerance("FinalizeSolution");
    ProcessInfo::getInstance().tasks->addTask(tCheckConstrTol, "CheckConstrTol");

    TaskBase *tCheckIterError = new TaskCheckIterationError("FinalizeSolution");
    ProcessInfo::getInstance().tasks->addTask(tCheckIterError, "CheckIterError");

    TaskBase *tCheckObjStag = new TaskCheckObjectiveStagnation("FinalizeSolution");
    ProcessInfo::getInstance().tasks->addTask(tCheckObjStag, "CheckObjStag");

    if (Settings::getInstance().getBoolSetting("FixedInteger.Use", "Dual"))
    {
        TaskBase *tSolveFixedLP = new TaskSolveFixedDualProblem(MIPSolver);
        ProcessInfo::getInstance().tasks->addTask(tSolveFixedLP, "SolveFixedLP");
        ProcessInfo::getInstance().tasks->addTask(tCheckAbsGap, "CheckAbsGap");
        ProcessInfo::getInstance().tasks->addTask(tCheckRelGap, "CheckRelGap");
    }

    if (Settings::getInstance().getBoolSetting("FixedInteger.Use", "Primal") && ProcessInfo::getInstance().originalProblem->getNumberOfNonlinearConstraints() > 0 && ProcessInfo::getInstance().originalProblem->getNumberOfDiscreteVariables() > 0)
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

    ProcessInfo::getInstance().tasks->addTask(tInitializeIteration, "InitIter");

    ProcessInfo::getInstance().tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategy");

    TaskBase *tExecuteSolLimStrategy = new TaskExecuteSolutionLimitStrategy(MIPSolver);
    ProcessInfo::getInstance().tasks->addTask(tExecuteSolLimStrategy, "ExecSolLimStrategy");

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

    ProcessInfo::getInstance().tasks->addTask(tAddHPs, "AddHPs");

    if (Settings::getInstance().getBoolSetting("HyperplaneCuts.UseIntegerCuts", "Dual"))
    {
        TaskBase *tAddICs = new TaskAddIntegerCuts(MIPSolver);
        ProcessInfo::getInstance().tasks->addTask(tAddICs, "AddICs");
    }

    TaskBase *tGoto = new TaskGoto("SolveIter");
    ProcessInfo::getInstance().tasks->addTask(tGoto, "Goto");

    ProcessInfo::getInstance().tasks->addTask(tFinalizeSolution, "FinalizeSolution");
}

SolutionStrategyMultiTree::~SolutionStrategyMultiTree()
{
}

bool SolutionStrategyMultiTree::solveProblem()
{
    TaskBase *nextTask;

    while (ProcessInfo::getInstance().tasks->getNextTask(nextTask))
    {
        Output::getInstance().outputInfo("┌─── Started task:  " + nextTask->getType());
        nextTask->run();
        Output::getInstance().outputInfo("└─── Finished task: " + nextTask->getType());
    }

    return (true);
}

void SolutionStrategyMultiTree::initializeStrategy()
{
}
