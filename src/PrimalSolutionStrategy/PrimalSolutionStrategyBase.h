/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../PrimalSolutionStrategy/IPrimalSolutionStrategy.h"
#include "SHOTSettings.h"
#include "../ProcessInfo.h"
#include "../OptProblems/OptProblemOriginal.h"

namespace SHOT
{
class PrimalSolutionStrategyBase : public IPrimalSolutionStrategy
{
  public:
    //PrimalSolutionStrategyBase(EnvironmentPtr envPtr);
    virtual ~PrimalSolutionStrategyBase();

  protected:
    EnvironmentPtr env;

  private:
};
} // namespace SHOT