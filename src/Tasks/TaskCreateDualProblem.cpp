/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCreateDualProblem.h"

#include "../DualSolver.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"

#include "../MIPSolver/IMIPSolver.h"
#include "../Model/Problem.h"

#include "TaskCreateMIPProblem.h"

namespace SHOT
{

TaskCreateDualProblem::TaskCreateDualProblem(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    // Reading and reformulating a large problem can use up the time limit, and creating the MIP problem then only
    // adds to the overrun, e.g., several seconds for millions of rows, so the solution is finalized directly instead
    if(env->timing->getElapsedTime("Total") >= env->settings->getSetting<double>("Termination.TimeLimit"))
    {
        isTimeLimitReachedBeforeCreation = true;
        env->output->outputDebug(" Dual problem not created since the time limit has been reached");
        return;
    }

    env->timing->startTimer("DualStrategy");

    env->output->outputDebug(" Creating dual problem");

    taskCreateMIPProblem
        = std::make_shared<TaskCreateMIPProblem>(env, env->dualSolver->MIPSolver, env->reformulatedProblem);

    taskCreateMIPProblem->run();
    env->dualSolver->MIPSolver->initializeSolverSettings();

    env->output->outputDebug(" Dual problem created");
    env->timing->stopTimer("DualStrategy");
}

TaskCreateDualProblem::~TaskCreateDualProblem() = default;

void TaskCreateDualProblem::run()
{
    // An iteration is created, since the results refer to the current iteration, and the solution is finalized as
    // when the time limit is reached in an iteration, so that the results and solution files are written as usual
    if(isTimeLimitReachedBeforeCreation)
    {
        env->results->createIteration();
        env->results->terminationReason = E_TerminationReason::TimeLimit;
        env->results->terminationReasonDescription = "Terminated since time limit was reached.";
        env->tasks->setNextTask("FinalizeSolution");
        return;
    }

    // Only run this task after intialization if we want to rebuild the tree in the multi-tree strategy
    if(env->settings->getSetting<bool>("Dual.TreeStrategy.Multi.Reinitialize"))
    {
        env->timing->startTimer("DualStrategy");

        env->output->outputDebug("        Recreating dual problem");

        taskCreateMIPProblem->run();

        if(env->settings->getSetting<bool>("Output.Debug.Enable"))
        {
            env->dualSolver->MIPSolver->writeProblemToDebugFiles(
                env->settings->getSetting<std::string>("Output.Debug.Path") + "/lp0");
        }

        env->output->outputDebug("        Dual problem recreated");
        env->timing->stopTimer("DualStrategy");
    }
}

std::string TaskCreateDualProblem::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT