/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

namespace SHOT
{
class TaskCheckUserTermination : public TaskBase
{
public:
    /// With invokeCallbacks false, the task only checks whether termination has been requested earlier, e.g., by a
    /// callback at another location, and does not check whether the dual solver was interrupted
    TaskCheckUserTermination(EnvironmentPtr envPtr, std::string taskIDTrue, bool invokeCallbacks = true);
    ~TaskCheckUserTermination() override;

    void run() override;
    std::string getType() override;

private:
    std::string taskIDIfTrue;
    bool invokeCallbacks;
};
} // namespace SHOT