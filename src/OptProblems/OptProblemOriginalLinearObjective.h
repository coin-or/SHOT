/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "OptProblemOriginal.h"

class OptProblemOriginalLinearObjective : public OptProblemOriginal
{
  public:
    OptProblemOriginalLinearObjective(EnvironmentPtr envPtr);
    ~OptProblemOriginalLinearObjective();

    virtual bool setProblem(OSInstance *instance);

  private:
};
