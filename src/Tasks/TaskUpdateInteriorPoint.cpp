/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskUpdateInteriorPoint.h"

#include "../DualSolver.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"

#include "../Model/Problem.h"

namespace SHOT
{

TaskUpdateInteriorPoint::TaskUpdateInteriorPoint(EnvironmentPtr envPtr) : TaskBase(envPtr) { }

TaskUpdateInteriorPoint::~TaskUpdateInteriorPoint() = default;

void TaskUpdateInteriorPoint::run()
{
    // If we do not yet have a valid primal solution we can't do anything
    if(!env->results->hasPrimalSolution())
        return;

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints == 0)
        return;

    env->timing->startTimer("InteriorPointSearch");

    auto maxDevPrimal = env->results->primalSolutions.at(0).maxDevatingConstraintNonlinear;
    auto tmpPrimalPoint = env->results->primalSolutions.at(0).point;

    // If we do not have an interior point, but uses the ESH dual strategy, update with primal solution
    if(env->dualSolver->interiorPts.size() == 0 && maxDevPrimal.value < 0)
    {
        auto tmpIP = std::make_shared<InteriorPoint>();

        if((int)tmpPrimalPoint.size() < env->reformulatedProblem->properties.numberOfVariables)
            env->reformulatedProblem->augmentAuxiliaryVariableValues(tmpPrimalPoint);

        assert(tmpPrimalPoint.size() == env->reformulatedProblem->properties.numberOfVariables);

        tmpIP->point = tmpPrimalPoint;
        assert((int)tmpIP->point.size() == env->reformulatedProblem->properties.numberOfVariables);

        auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
            tmpIP->point, env->reformulatedProblem->nonlinearConstraints);
        tmpIP->maxDevatingConstraint = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);

        env->output->outputDebug("     Interior point replaced with primal solution point since no interior point was "
                                 "previously available.");

        env->dualSolver->interiorPts.push_back(tmpIP);

        env->timing->stopTimer("InteriorPointSearch");
        return;
    }
    else if(env->dualSolver->interiorPts.size() == 0)
    {
        env->timing->stopTimer("InteriorPointSearch");
        return;
    }

    // Need to calculate the value for the point in the reformulated problem
    auto tmpIP = std::make_shared<InteriorPoint>();

    if((int)tmpPrimalPoint.size() < env->reformulatedProblem->properties.numberOfVariables)
        env->reformulatedProblem->augmentAuxiliaryVariableValues(tmpPrimalPoint);

    assert(tmpPrimalPoint.size() == env->reformulatedProblem->properties.numberOfVariables);

    tmpIP->point = tmpPrimalPoint;

    auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
        tmpIP->point, env->reformulatedProblem->nonlinearConstraints);
    tmpIP->maxDevatingConstraint = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);

    // Replace the current point with the new point if it is deeper within the feasible region
    if(maxDev.normalizedValue < env->dualSolver->interiorPts.at(0)->maxDevatingConstraint.value)
    {
        env->output->outputDebug(
            "     Interior point replaced with primal solution point due to constraint deviation.");

        env->dualSolver->interiorPts.back() = tmpIP;
    }
    // Add the new point
    else if(env->settings->getSetting<int>("ESH.InteriorPoint.UsePrimalSolution", "Dual")
            == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepBoth)
        && maxDev.normalizedValue < 0)
    {
        env->output->outputDebug("        Primal solution point used as additional interior point.");

        if((int)env->dualSolver->interiorPts.size() == env->solutionStatistics.numberOfOriginalInteriorPoints)
        {
            env->dualSolver->interiorPts.push_back(tmpIP);
        }
        else
        {
            env->dualSolver->interiorPts.back() = tmpIP;
        }
    }
    // Use the new point only
    else if(env->settings->getSetting<int>("ESH.InteriorPoint.UsePrimalSolution", "Dual")
            == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepNew)
        && maxDev.normalizedValue < 0)
    {
        env->output->outputDebug("     Interior point replaced with primal solution point.");

        env->dualSolver->interiorPts.back() = tmpIP;
    }
    // Find a new point in the midpoint between the original and new
    else if(env->settings->getSetting<int>("ESH.InteriorPoint.UsePrimalSolution", "Dual")
            == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::OnlyAverage)
        && maxDev.normalizedValue < 0)
    {
        auto tmpIP = std::make_shared<InteriorPoint>();

        for(size_t i = 0; i < tmpPrimalPoint.size(); i++)
            tmpPrimalPoint.at(i) = (0.5 * tmpPrimalPoint.at(i) + 0.5 * env->dualSolver->interiorPts.at(0)->point.at(i));

        if((int)tmpPrimalPoint.size() < env->reformulatedProblem->properties.numberOfVariables)
            env->reformulatedProblem->augmentAuxiliaryVariableValues(tmpPrimalPoint);

        assert(tmpPrimalPoint.size() == env->reformulatedProblem->properties.numberOfVariables);

        tmpIP->point = tmpPrimalPoint;

        auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
            tmpIP->point, env->reformulatedProblem->nonlinearConstraints);
        tmpIP->maxDevatingConstraint = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);

        env->output->outputDebug("     Interior point replaced with primal solution point.");

        env->dualSolver->interiorPts.back() = tmpIP;
    }

    env->timing->stopTimer("InteriorPointSearch");
}

std::string TaskUpdateInteriorPoint::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT