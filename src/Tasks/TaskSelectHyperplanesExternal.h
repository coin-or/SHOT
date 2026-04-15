/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#include "../Structs.h"

namespace SHOT
{
class TaskSelectHyperplanesExternal : public TaskBase
{
public:
    TaskSelectHyperplanesExternal(EnvironmentPtr envPtr);
    ~TaskSelectHyperplanesExternal() override;

    void run() override;
    virtual void run(std::vector<SolutionPoint> solPoints);

    std::string getType() override;

private:
};
} // namespace SHOT