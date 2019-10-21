/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "../Environment.h"

namespace SHOT
{
class ISolutionStrategy
{
public:
    virtual ~ISolutionStrategy() = default;

    virtual void initializeStrategy() = 0;
    virtual bool solveProblem() = 0;

protected:
    EnvironmentPtr env;
};
} // namespace SHOT
