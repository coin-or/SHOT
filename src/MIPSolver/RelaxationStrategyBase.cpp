/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "RelaxationStrategyBase.h"

bool RelaxationStrategyBase::isRelaxedSolutionEpsilonValid()
{
    auto prevIter = env->process->getPreviousIteration();

    if (prevIter->maxDeviation < env->settings->getDoubleSetting("ConstraintTolerance", "Termination"))
    {
        return true;
    }

    return false;
}

bool RelaxationStrategyBase::isRelaxedSolutionInterior()
{
    auto prevIter = env->process->getPreviousIteration();

    if (prevIter->maxDeviation < 0)
    {
        return true;
    }

    return false;
}

bool RelaxationStrategyBase::isCurrentToleranceReached()
{
    auto prevIter = env->process->getPreviousIteration();

    if (prevIter->maxDeviation < env->settings->getDoubleSetting("Relaxation.TerminationTolerance", "Dual"))
    {
        return true;
    }

    return false;
}

bool RelaxationStrategyBase::isGapReached()
{
    auto prevIter = env->process->getPreviousIteration();

    if (env->process->getAbsoluteObjectiveGap() < 2 * env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination"))
    {
        return true;
    }

    if (env->process->getRelativeObjectiveGap() < 2 * env->settings->getDoubleSetting("ObjectiveGap.Relative", "Termination"))
    {
        return true;
    }

    return false;
}
