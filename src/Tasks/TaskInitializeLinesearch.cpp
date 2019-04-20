/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskInitializeLinesearch.h"

#include "../Timing.h"

#include "../LinesearchMethod/LinesearchMethodBoost.h"

namespace SHOT
{

TaskInitializeLinesearch::TaskInitializeLinesearch(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("DualCutGenerationRootSearch");

    env->rootsearchMethod = std::dynamic_pointer_cast<ILinesearchMethod>(std::make_shared<LinesearchMethodBoost>(env));

    env->timing->stopTimer("DualCutGenerationRootSearch");
}

TaskInitializeLinesearch::~TaskInitializeLinesearch() {}

void TaskInitializeLinesearch::run() {}

std::string TaskInitializeLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT