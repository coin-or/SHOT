/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskUpdateInteriorPoint.h"

TaskUpdateInteriorPoint::TaskUpdateInteriorPoint(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
}

TaskUpdateInteriorPoint::~TaskUpdateInteriorPoint()
{
}

void TaskUpdateInteriorPoint::run()
{
    // If we do not yet have a valid primal solution we can't do anything
    if (env->process->primalSolutions.size() == 0)
        return;

    env->process->startTimer("InteriorPointSearch");

    auto maxDevPrimal = env->process->primalSolutions.at(0).maxDevatingConstraintNonlinear;
    auto tmpPrimalPoint = env->process->primalSolutions.at(0).point;

    // If we do not have an interior point, but uses the ESH dual strategy, update with primal solution
    if (env->process->interiorPts.size() == 0 && maxDevPrimal.value < 0)
    {
        std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());
        tmpIP->point = env->process->primalSolutions.at(0).point;
        tmpIP->maxDevatingConstraint = env->process->primalSolutions.at(0).maxDevatingConstraintNonlinear;

        env->output->outputInfo("     Interior point replaced with primal solution point since no interior point was previously available.");

        env->process->interiorPts.push_back(tmpIP);

        env->process->stopTimer("InteriorPointSearch");
        return;
    }
    else if (env->process->interiorPts.size() == 0)
    {
        env->process->stopTimer("InteriorPointSearch");
        return;
    }

    // Add the new point if it is deeper within the feasible region
    if (maxDevPrimal.value < env->process->interiorPts.at(0)->maxDevatingConstraint.value)
    {
        std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());
        tmpIP->point = tmpPrimalPoint;
        tmpIP->maxDevatingConstraint = maxDevPrimal;

        env->output->outputInfo("     Interior point replaced with primal solution point due to constraint deviation.");

        env->process->interiorPts.back() = tmpIP;
    }
    else if (env->settings->getIntSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual") == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepBoth) && maxDevPrimal.value < 0)
    {
        std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());

        tmpIP->point = tmpPrimalPoint;
        tmpIP->maxDevatingConstraint = maxDevPrimal;

        env->output->outputInfo("     Primal solution point used as additional interior point.");

        if (env->process->interiorPts.size() == env->solutionStatistics.numberOfOriginalInteriorPoints)
        {
            env->process->interiorPts.push_back(tmpIP);
        }
        else
        {
            env->process->interiorPts.back() = tmpIP;
        }
    }
    else if (env->settings->getIntSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual") == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepNew) && maxDevPrimal.value < 0)
    {
        std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());

        // Add the new point only
        tmpIP->point = tmpPrimalPoint;
        tmpIP->maxDevatingConstraint = maxDevPrimal;

        env->output->outputInfo("     Interior point replaced with primal solution point.");

        env->process->interiorPts.back() = tmpIP;
    }
    else if (env->settings->getIntSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual") == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::OnlyAverage) && maxDevPrimal.value < 0)
    {
        std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());

        // Find a new point in the midpoint between the original and new
        for (int i = 0; i < tmpPrimalPoint.size(); i++)
        {
            tmpPrimalPoint.at(i) = (0.5 * tmpPrimalPoint.at(i) + 0.5 * env->process->interiorPts.at(0)->point.at(i));
        }

        tmpIP->point = tmpPrimalPoint;
        tmpIP->maxDevatingConstraint = env->model->originalProblem->getMostDeviatingConstraint(tmpPrimalPoint);

        env->output->outputInfo("     Interior point replaced with primal solution point.");

        env->process->interiorPts.back() = tmpIP;
    }

    env->process->stopTimer("InteriorPointSearch");
}

std::string TaskUpdateInteriorPoint::getType()
{
    std::string type = typeid(this).name();
    return (type);
}