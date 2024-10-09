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
#include "../Settings.h"
#include "../Timing.h"

#include "../MIPSolver/IMIPSolver.h"
#include "../Model/Problem.h"

#include "TaskCreateMIPProblem.h"

namespace SHOT
{

TaskCreateDualProblem::TaskCreateDualProblem(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
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
    // Only run this task after intialization if we want to rebuild the tree in the multi-tree strategy
    if(env->settings->getSetting<bool>("TreeStrategy.Multi.Reinitialize", "Dual"))
    {
        env->timing->startTimer("DualStrategy");

        env->output->outputDebug("        Recreating dual problem");

        taskCreateMIPProblem->run();

        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            env->dualSolver->MIPSolver->writeProblemToFile(
                env->settings->getSetting<std::string>("Debug.Path", "Output") + "/lp0.lp");
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