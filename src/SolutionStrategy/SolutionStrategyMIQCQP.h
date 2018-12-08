/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "ISolutionStrategy.h"

namespace SHOT
{
class SolutionStrategyMIQCQP : public ISolutionStrategy
{
  public:
    SolutionStrategyMIQCQP(EnvironmentPtr envPtr, OSInstance *osInstance);
    virtual ~SolutionStrategyMIQCQP();

    virtual bool solveProblem();
    virtual void initializeStrategy();

  protected:
};
} // namespace SHOT