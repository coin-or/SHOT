/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskPresolve.h"

namespace SHOT
{

TaskPresolve::TaskPresolve(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->process->startTimer("DualStrategy");

    isPresolved = false;

    env->process->stopTimer("DualStrategy");
}

TaskPresolve::~TaskPresolve()
{
}

void TaskPresolve::run()
{
    env->process->startTimer("DualStrategy");
    auto currIter = env->process->getCurrentIteration();

    auto strategy = static_cast<ES_MIPPresolveStrategy>(env->settings->getIntSetting("MIP.Presolve.Frequency", "Dual"));

    if (!currIter->isMIP())
    {
        env->process->stopTimer("DualStrategy");
        return;
    }

    if (strategy == ES_MIPPresolveStrategy::Never)
    {
        env->process->stopTimer("DualStrategy");
        return;
    }
    else if (strategy == ES_MIPPresolveStrategy::Once && isPresolved == true)
    {
        env->process->stopTimer("DualStrategy");
        return;
    }

    // Sets the iteration time limit
    auto timeLim = env->settings->getDoubleSetting("TimeLimit", "Termination") - env->process->getElapsedTime("Total");
    env->dualSolver->setTimeLimit(timeLim);

    if (env->process->primalSolutions.size() > 0)
    {
        env->dualSolver->setCutOff(env->process->getPrimalBound());
    }

    if (env->dualSolver->getDiscreteVariableStatus() && env->process->primalSolutions.size() > 0)
    {
        env->dualSolver->addMIPStart(env->process->primalSolution);
    }

    if (env->settings->getBoolSetting("FixedInteger.UsePresolveBounds", "Primal") || env->settings->getBoolSetting("MIP.Presolve.UpdateObtainedBounds", "Dual"))
    {
        env->dualSolver->presolveAndUpdateBounds();
        isPresolved = true;
    }

    env->process->stopTimer("DualStrategy");
}

std::string TaskPresolve::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT