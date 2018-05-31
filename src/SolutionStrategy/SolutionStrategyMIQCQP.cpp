/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "SolutionStrategyMIQCQP.h"

SolutionStrategyMIQCQP::SolutionStrategyMIQCQP(OSInstance *osInstance)
{
    ProcessInfo::getInstance().createTimer("ProblemInitialization", " - problem initialization");
    ProcessInfo::getInstance().createTimer("InteriorPointSearch", " - interior point search");

    ProcessInfo::getInstance().createTimer("DualStrategy", " - dual strategy");
    ProcessInfo::getInstance().createTimer("DualProblemsDiscrete", "   - solving MIP problems");

    ProcessInfo::getInstance().createTimer("PrimalStrategy", " - primal strategy");

    auto solverMIP = static_cast<ES_MIPSolver>(Settings::getInstance().getIntSetting("MIP.Solver", "Dual"));

    TaskBase *tFinalizeSolution = new TaskSequential();

    TaskBase *tInitMIPSolver = new TaskInitializeDualSolver(solverMIP, false);
    ProcessInfo::getInstance().tasks->addTask(tInitMIPSolver, "InitMIPSolver");

    auto MIPSolver = ProcessInfo::getInstance().MIPSolver;

    TaskBase *tInitOrigProblem = new TaskInitializeOriginalProblem(osInstance);
    ProcessInfo::getInstance().tasks->addTask(tInitOrigProblem, "InitOrigProb");

    TaskBase *tCreateDualProblem = new TaskCreateDualProblem(MIPSolver);
    ProcessInfo::getInstance().tasks->addTask(tCreateDualProblem, "CreateDualProblem");

    TaskBase *tInitializeIteration = new TaskInitializeIteration();
    ProcessInfo::getInstance().tasks->addTask(tInitializeIteration, "InitIter");


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

    TaskBase *tCheckTimeLim = new TaskCheckTimeLimit("FinalizeSolution");
    ProcessInfo::getInstance().tasks->addTask(tCheckTimeLim, "CheckTimeLim");

    TaskBase *tCheckIterLim = new TaskCheckIterationLimit("FinalizeSolution");
    ProcessInfo::getInstance().tasks->addTask(tCheckIterLim, "CheckIterLim");

    TaskBase *tCheckConstrTol = new TaskCheckConstraintTolerance("FinalizeSolution");
    ProcessInfo::getInstance().tasks->addTask(tCheckConstrTol, "CheckConstrTol");

    ProcessInfo::getInstance().tasks->addTask(tFinalizeSolution, "FinalizeSolution");
}

SolutionStrategyMIQCQP::~SolutionStrategyMIQCQP()
{
}

bool SolutionStrategyMIQCQP::solveProblem()
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

void SolutionStrategyMIQCQP::initializeStrategy()
{
}
