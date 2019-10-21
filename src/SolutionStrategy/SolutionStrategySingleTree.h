/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "ISolutionStrategy.h"
#include "../Environment.h"

namespace SHOT
{
class SolutionStrategySingleTree : public ISolutionStrategy
{
public:
    SolutionStrategySingleTree(EnvironmentPtr envPtr);
    virtual ~SolutionStrategySingleTree();

    bool solveProblem() override;
    void initializeStrategy() override;

protected:
};
} // namespace SHOT