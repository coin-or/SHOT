/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "ProcessInfo.h"
#include "Environment.h"
#include "TaskHandler.h"

class ISolutionStrategy
{
  public:
    ~ISolutionStrategy();

    virtual void initializeStrategy() = 0;
    virtual bool solveProblem() = 0;

  protected:
    EnvironmentPtr env;
};