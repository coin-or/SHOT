/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCalculateSolutionChangeNorm.h"

namespace SHOT
{

TaskCalculateSolutionChangeNorm::TaskCalculateSolutionChangeNorm(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
}

TaskCalculateSolutionChangeNorm::~TaskCalculateSolutionChangeNorm()
{
}

void TaskCalculateSolutionChangeNorm::run()
{
    auto currIter = env->process->getCurrentIteration();

    currIter->boundaryDistance = SHOT_DBL_MAX;

    if (env->process->iterations.size() < 3)
    {
        return;
    }

    if (env->process->getCurrentIteration()->hyperplanePoints.size() == 0 || env->process->getCurrentIteration()->isMIP())
    {
        return;
    }

    auto currIterSol = env->process->getCurrentIteration()->hyperplanePoints.at(0);

    for (int i = env->process->iterations.size() - 2; i >= 1; i--)
    {
        if (env->process->iterations.size() > 0 && !env->process->iterations.at(i).isMIP())
        {
            auto prevIterSol = env->process->iterations.at(i).hyperplanePoints.at(0);

            double distance = 0;

            for (int j = 0; j < currIterSol.size(); j++)
            {
                distance = distance + (currIterSol.at(j) - prevIterSol.at(j)) * (currIterSol.at(j) - prevIterSol.at(j));
            }

            distance = sqrt(distance + 0.001);

            if (OSIsnan(distance)) // Checks for INF, do not remove!
            {
                currIter->boundaryDistance = SHOT_DBL_MAX;
            }
            else
            {
                currIter->boundaryDistance = distance;
            }

            return;
        }
    }
}

std::string TaskCalculateSolutionChangeNorm::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT