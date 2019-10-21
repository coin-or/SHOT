/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskInitializeRootsearch.h"

#include "../Timing.h"

#include "../RootsearchMethod/RootsearchMethodBoost.h"

namespace SHOT
{

TaskInitializeRootsearch::TaskInitializeRootsearch(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("DualCutGenerationRootSearch");

    env->rootsearchMethod = std::dynamic_pointer_cast<IRootsearchMethod>(std::make_shared<RootsearchMethodBoost>(env));

    env->timing->stopTimer("DualCutGenerationRootSearch");
}

TaskInitializeRootsearch::~TaskInitializeRootsearch() = default;

void TaskInitializeRootsearch::run() {}

std::string TaskInitializeRootsearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT