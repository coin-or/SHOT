/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "OptProblem.h"
#include "../UtilityFunctions.h"

class OptProblemOriginal : public OptProblem
{
  public:
    OptProblemOriginal();
    ~OptProblemOriginal();

    virtual bool setProblem(OSInstance *instance) = 0;
};
