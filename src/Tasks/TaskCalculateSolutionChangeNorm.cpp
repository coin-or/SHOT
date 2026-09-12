/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCalculateSolutionChangeNorm.h"

#include "../Iteration.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Utilities.h"

#include <cmath>

namespace SHOT
{

TaskCalculateSolutionChangeNorm::TaskCalculateSolutionChangeNorm(EnvironmentPtr envPtr) : TaskBase(envPtr) {}

TaskCalculateSolutionChangeNorm::~TaskCalculateSolutionChangeNorm() = default;

void TaskCalculateSolutionChangeNorm::run()
{
    auto currIter = env->results->getCurrentIteration();

    currIter->boundaryDistance = SHOT_DBL_MAX;

    if(env->results->getNumberOfIterations() < 3)
        return;

    if(currIter->hyperplanePoints.size() == 0)
        return;

    auto currIterSol = currIter->hyperplanePoints.at(0);

    // The distance is calculated to the last iteration that also generated a hyperplane, whether the dual
    // problem was a relaxation there or not
    for(int i = env->results->getNumberOfIterations() - 2; i >= 1; i--)
    {
        auto previousIteration = env->results->iterations.at(i);

        if(previousIteration->hyperplanePoints.size() == 0)
            continue;

        auto prevIterSol = previousIteration->hyperplanePoints.at(0);

        if(prevIterSol.size() != currIterSol.size())
            continue;

        double distance = Utilities::L2Norm(currIterSol, prevIterSol);

        // Checks for INF, do not remove!
        currIter->boundaryDistance = std::isnan(distance) ? SHOT_DBL_MAX : distance;

        env->output->outputDebug(fmt::format("        The hyperplane generation point moved {} from iteration {}.",
            currIter->boundaryDistance, previousIteration->iterationNumber));

        return;
    }
}

std::string TaskCalculateSolutionChangeNorm::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT