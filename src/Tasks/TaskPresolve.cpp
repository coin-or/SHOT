/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskPresolve.h"

#include "../DualSolver.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"

#include "../MIPSolver/IMIPSolver.h"

#include "../Model/Problem.h"

namespace SHOT
{

TaskPresolve::TaskPresolve(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("DualStrategy");

    isPresolved = false;

    env->timing->stopTimer("DualStrategy");
}

TaskPresolve::~TaskPresolve() = default;

void TaskPresolve::run()
{
    env->timing->startTimer("DualStrategy");
    auto currIter = env->results->getCurrentIteration();

    auto strategy
        = static_cast<ES_MIPPresolveStrategy>(env->settings->getSetting<int>("MIP.Presolve.Frequency", "Dual"));

    if(!currIter->isMIP())
    {
        env->timing->stopTimer("DualStrategy");
        return;
    }

    if(strategy == ES_MIPPresolveStrategy::Never)
    {
        env->timing->stopTimer("DualStrategy");
        return;
    }
    else if(strategy == ES_MIPPresolveStrategy::Once && isPresolved == true)
    {
        env->timing->stopTimer("DualStrategy");
        return;
    }

    // Sets the iteration time limit
    auto timeLim = env->settings->getSetting<double>("TimeLimit", "Termination") - env->timing->getElapsedTime("Total");
    env->dualSolver->MIPSolver->setTimeLimit(timeLim);

    if(env->results->hasPrimalSolution())
    {
        // env->dualSolver->MIPSolver->setCutOff(env->results->getPrimalBound());
    }

    if(env->dualSolver->MIPSolver->getDiscreteVariableStatus() && env->results->hasPrimalSolution())
    {
        env->dualSolver->MIPSolver->addMIPStart(env->results->primalSolution);
    }

    if(env->settings->getSetting<bool>("FixedInteger.UsePresolveBounds", "Primal")
        || env->settings->getSetting<bool>("MIP.Presolve.UpdateObtainedBounds", "Dual"))
    {
        env->dualSolver->MIPSolver->presolveAndUpdateBounds();
        isPresolved = true;
    }

    env->timing->stopTimer("DualStrategy");
}

std::string TaskPresolve::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT