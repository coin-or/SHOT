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

#include <vector>

namespace SHOT
{
class TaskSelectHyperplanePointsObjectiveFunction : public TaskBase
{
public:
    TaskSelectHyperplanePointsObjectiveFunction(EnvironmentPtr envPtr);
    ~TaskSelectHyperplanePointsObjectiveFunction() override;

    void run() override;
    virtual void run(std::vector<SolutionPoint> solPoints);
    std::string getType() override;
};
} // namespace SHOT