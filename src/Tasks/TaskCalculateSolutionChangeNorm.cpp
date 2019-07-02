/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCalculateSolutionChangeNorm.h"

#include "../Iteration.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"

namespace SHOT
{

TaskCalculateSolutionChangeNorm::TaskCalculateSolutionChangeNorm(EnvironmentPtr envPtr) : TaskBase(envPtr) {}

TaskCalculateSolutionChangeNorm::~TaskCalculateSolutionChangeNorm() = default;

void TaskCalculateSolutionChangeNorm::run()
{
    auto currIter = env->results->getCurrentIteration();

    currIter->boundaryDistance = SHOT_DBL_MAX;

    if(env->results->getNumberOfIterations() < 3)
    {
        return;
    }

    if(env->results->getCurrentIteration()->hyperplanePoints.size() == 0
        || env->results->getCurrentIteration()->isMIP())
    {
        return;
    }

    auto currIterSol = env->results->getCurrentIteration()->hyperplanePoints.at(0);

    for(int i = env->results->getNumberOfIterations() - 2; i >= 1; i--)
    {
        if(env->results->getNumberOfIterations() > 0 && !env->results->iterations.at(i)->isMIP())
        {
            auto prevIterSol = env->results->iterations.at(i)->hyperplanePoints.at(0);

            double distance = 0;

            for(size_t j = 0; j < currIterSol.size(); j++)
            {
                distance = distance + (currIterSol.at(j) - prevIterSol.at(j)) * (currIterSol.at(j) - prevIterSol.at(j));
            }

            distance = sqrt(distance + 0.001);

            if(std::isnan(distance)) // Checks for INF, do not remove!
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