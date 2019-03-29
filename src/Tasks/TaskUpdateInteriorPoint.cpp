/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskUpdateInteriorPoint.h"

namespace SHOT
{

TaskUpdateInteriorPoint::TaskUpdateInteriorPoint(EnvironmentPtr envPtr) : TaskBase(envPtr) {}

TaskUpdateInteriorPoint::~TaskUpdateInteriorPoint() {}

void TaskUpdateInteriorPoint::run()
{
    // If we do not yet have a valid primal solution we can't do anything
    if(env->results->primalSolutions.size() == 0)
        return;

    env->timing->startTimer("InteriorPointSearch");

    auto maxDevPrimal = env->results->primalSolutions.at(0).maxDevatingConstraintNonlinear;
    auto tmpPrimalPoint = env->results->primalSolutions.at(0).point;

    // If we do not have an interior point, but uses the ESH dual strategy, update with primal solution
    if(env->dualSolver->MIPSolver->interiorPts.size() == 0 && maxDevPrimal.value < 0)
    {
        auto tmpIP = std::make_shared<InteriorPoint>();
        tmpIP->point = env->results->primalSolutions.at(0).point;
        tmpIP->maxDevatingConstraint = env->results->primalSolutions.at(0).maxDevatingConstraintNonlinear;

        env->output->outputDebug("     Interior point replaced with primal solution point since no interior point was "
                                "previously available.");

        env->dualSolver->MIPSolver->interiorPts.push_back(tmpIP);

        env->timing->stopTimer("InteriorPointSearch");
        return;
    }
    else if(env->dualSolver->MIPSolver->interiorPts.size() == 0)
    {
        env->timing->stopTimer("InteriorPointSearch");
        return;
    }

    // Add the new point if it is deeper within the feasible region
    if(maxDevPrimal.value < env->dualSolver->MIPSolver->interiorPts.at(0)->maxDevatingConstraint.value)
    {
        auto tmpIP = std::make_shared<InteriorPoint>();
        tmpIP->point = tmpPrimalPoint;
        tmpIP->maxDevatingConstraint = maxDevPrimal;

        env->output->outputDebug("     Interior point replaced with primal solution point due to constraint deviation.");

        env->dualSolver->MIPSolver->interiorPts.back() = tmpIP;
    }
    else if(env->settings->getSetting<int>("ESH.InteriorPoint.UsePrimalSolution", "Dual")
            == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepBoth)
        && maxDevPrimal.value < 0)
    {
        auto tmpIP = std::make_shared<InteriorPoint>();

        tmpIP->point = tmpPrimalPoint;
        tmpIP->maxDevatingConstraint = maxDevPrimal;

        env->output->outputDebug("     Primal solution point used as additional interior point.");

        if(env->dualSolver->MIPSolver->interiorPts.size() == env->solutionStatistics.numberOfOriginalInteriorPoints)
        {
            env->dualSolver->MIPSolver->interiorPts.push_back(tmpIP);
        }
        else
        {
            env->dualSolver->MIPSolver->interiorPts.back() = tmpIP;
        }
    }
    else if(env->settings->getSetting<int>("ESH.InteriorPoint.UsePrimalSolution", "Dual")
            == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepNew)
        && maxDevPrimal.value < 0)
    {
        auto tmpIP = std::make_shared<InteriorPoint>();

        // Add the new point only
        tmpIP->point = tmpPrimalPoint;
        tmpIP->maxDevatingConstraint = maxDevPrimal;

        env->output->outputDebug("     Interior point replaced with primal solution point.");

        env->dualSolver->MIPSolver->interiorPts.back() = tmpIP;
    }
    else if(env->settings->getSetting<int>("ESH.InteriorPoint.UsePrimalSolution", "Dual")
            == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::OnlyAverage)
        && maxDevPrimal.value < 0)
    {
        auto tmpIP = std::make_shared<InteriorPoint>();

        // Find a new point in the midpoint between the original and new
        for(int i = 0; i < tmpPrimalPoint.size(); i++)
        {
            tmpPrimalPoint.at(i)
                = (0.5 * tmpPrimalPoint.at(i) + 0.5 * env->dualSolver->MIPSolver->interiorPts.at(0)->point.at(i));
        }

        tmpIP->point = tmpPrimalPoint;

        auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
            tmpIP->point, env->reformulatedProblem->nonlinearConstraints);
        tmpIP->maxDevatingConstraint = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);

        env->output->outputDebug("     Interior point replaced with primal solution point.");

        env->dualSolver->MIPSolver->interiorPts.back() = tmpIP;
    }

    env->timing->stopTimer("InteriorPointSearch");
}

std::string TaskUpdateInteriorPoint::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT