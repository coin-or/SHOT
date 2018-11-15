/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCreateDualProblem.h"
namespace SHOT
{

TaskCreateDualProblem::TaskCreateDualProblem(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->process->startTimer("DualStrategy");

    env->output->outputDebug("Creating dual problem");

    env->dualSolver->createLinearProblem(env->model->originalProblem.get());

    env->dualSolver->initializeSolverSettings();

    if (env->settings->getBoolSetting("Debug.Enable", "Output"))
    {
        env->dualSolver->writeProblemToFile(env->settings->getStringSetting("Debug.Path", "Output") + "/lp0.lp");
    }

    env->output->outputDebug("Dual problem created");
    env->process->stopTimer("DualStrategy");
}

TaskCreateDualProblem::~TaskCreateDualProblem()
{
}

void TaskCreateDualProblem::run()
{
    if (env->settings->getBoolSetting("TreeStrategy.Multi.Reinitialize", "Dual"))
    {
        env->process->startTimer("DualStrategy");

        env->output->outputDebug("Creating dual problem");

        env->dualSolver->createLinearProblem(env->model->originalProblem.get());

        env->dualSolver->initializeSolverSettings();

        if (env->settings->getBoolSetting("Debug.Enable", "Output"))
        {
            env->dualSolver->writeProblemToFile(env->settings->getStringSetting("Debug.Path", "Output") + "/lp0.lp");
        }

        env->output->outputDebug("Dual problem created");
        env->process->stopTimer("DualStrategy");
    }
}

std::string TaskCreateDualProblem::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT