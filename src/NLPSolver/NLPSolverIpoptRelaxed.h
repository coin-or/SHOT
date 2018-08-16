/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "NLPSolverIpoptBase.h"
#include "../OptProblems/OptProblemNLPRelaxed.h"

class NLPSolverIpoptRelaxed : public NLPSolverBase, public NLPSolverIpoptBase
{
  public:
    NLPSolverIpoptRelaxed(EnvironmentPtr envPtr);
    virtual ~NLPSolverIpoptRelaxed();

    virtual DoubleVector getSolution();

  protected:
    bool createProblemInstance(OSInstance *origInstance);

    virtual void setSolverSpecificInitialSettings();

  private:
};
