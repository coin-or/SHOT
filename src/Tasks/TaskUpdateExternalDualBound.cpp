/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskUpdateExternalDualBound.h"

#include "../CallbackData.h"
#include "../Enums.h"
#include "../Environment.h"
#include "../EventHandler.h"
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
    // Check if external dual bound provider is available
    if(env->events->hasDataProvider(E_EventType::ExternalDualBound))
    {
        bool isMinimization = (env->reformulatedProblem->objectiveFunction->properties.isMinimize);
        double currentDualBound = env->results->getCurrentDualBound();
        double currentPrimalBound = env->results->getPrimalBound();
        int iterationNumber = env->results->getCurrentIteration()->iterationNumber;
        double absoluteGap = env->results->getAbsoluteCurrentObjectiveGap();
        double relativeGap = env->results->getRelativeCurrentObjectiveGap();

        std::optional<double> externalDualBound;

        // Create structured data for callback
        DualBoundCallbackData callbackData(isMinimization, currentDualBound, currentPrimalBound, iterationNumber,
            relativeGap, absoluteGap, env->solutionStatistics);

        // Pass the callback data and execute callback; retrieve the new dual bound
        externalDualBound = env->events->requestData<double>(E_EventType::ExternalDualBound, callbackData);

        if(double newBound = *externalDualBound; externalDualBound.has_value() && !std::isnan(newBound))
        {
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
            env->output->outputDebug("        External dual bound provider returned no value");
        }
    }
}

std::string TaskUpdateExternalDualBound::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

} // namespace SHOT
