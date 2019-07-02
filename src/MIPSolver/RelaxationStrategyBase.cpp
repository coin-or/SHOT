/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "RelaxationStrategyBase.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Results.h"
#include "../Settings.h"

#include "../Model/Problem.h"

namespace SHOT
{

bool RelaxationStrategyBase::isRelaxedSolutionInterior()
{
    if(env->results->getNumberOfIterations() < 2)
    {
        return false;
    }

    auto prevIter = env->results->getPreviousIteration();

    if(prevIter->maxDeviation < 0)
    {
        return true;
    }

    return false;
}

bool RelaxationStrategyBase::isConstraintToleranceReached()
{
    if(env->results->getNumberOfIterations() < 2)
    {
        return false;
    }

    auto prevIter = env->results->getPreviousIteration();

    double constraintTolerance = std::max(env->settings->getSetting<double>("ConstraintTolerance", "Termination"),
        env->settings->getSetting<double>("Relaxation.TerminationTolerance", "Dual"));

    if(prevIter->maxDeviation > constraintTolerance)
    {
        return false;
    }

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        if(env->reformulatedProblem->objectiveFunction->calculateValue(prevIter->solutionPoints.at(0).point)
                - prevIter->objectiveValue
            > constraintTolerance)
            return false;
    }

    return true;
}

bool RelaxationStrategyBase::isGapReached()
{
    if(env->results->getNumberOfIterations() < 2)
    {
        return false;
    }

    auto prevIter = env->results->getPreviousIteration();

    if(env->results->getAbsoluteGlobalObjectiveGap()
        < 2 * env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination"))
    {
        return true;
    }

    if(env->results->getRelativeGlobalObjectiveGap()
        < 2 * env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination"))
    {
        return true;
    }

    return false;
}
} // namespace SHOT