/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskUpdateExternalDualBound.h"

#include "../Callback.h"
#include "../Enums.h"
#include "../Environment.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"
#include "../Model/Problem.h"

namespace SHOT
{

TaskUpdateExternalDualBound::TaskUpdateExternalDualBound(EnvironmentPtr envPtr) : TaskBase(envPtr) { }

TaskUpdateExternalDualBound::~TaskUpdateExternalDualBound() = default;

void TaskUpdateExternalDualBound::run()
{
    if(env->callbacks->isActive(E_CallbackLocation::DualBoundUpdate))
    {
        bool isMinimization = (env->reformulatedProblem->objectiveFunction->properties.isMinimize);

        auto context = std::make_shared<DualBoundUpdateContext>(env);
        env->callbacks->invoke(*context);
        auto externalDualBound = context->getProposedDualBound();
        context->invalidate();

        double currentDualBound = env->results->getCurrentDualBound();

        if(externalDualBound.has_value())
        {
            double newBound = *externalDualBound;

            env->output->outputDebug(fmt::format("        External dual bound provider returned: {}", newBound));

            // For minimization problems, dual bound should be a lower bound
            // For maximization problems, dual bound should be an upper bound
            bool isBoundImproved = false;

            if(isMinimization)
            {
                // For minimization: new bound should be higher (better) than current
                isBoundImproved = (std::isnan(currentDualBound) || newBound > currentDualBound);
            }
            else
            {
                // For maximization: new bound should be lower (better) than current
                isBoundImproved = (std::isnan(currentDualBound) || newBound < currentDualBound);
            }

            if(isBoundImproved)
            {
                env->results->setDualBound(newBound);
                env->solutionStatistics.hasExternalDualBoundBeenSet = true;

                env->output->outputDebug(
                    fmt::format("        Updated dual bound from external provider: {}", newBound));
            }
            else
            {
                env->output->outputDebug(fmt::format(
                    "        External dual bound {} not better than current {}", newBound, currentDualBound));
            }
        }
        else
        {
            env->output->outputDebug("        No external dual bound was proposed");
        }
    }
}

std::string TaskUpdateExternalDualBound::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

} // namespace SHOT
