/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "NLPSolverIpoptBase.h"

namespace SHOT
{
class NLPSolverIpoptRelaxed : public NLPSolverBase, public NLPSolverIpoptBase
{
  public:
    NLPSolverIpoptRelaxed(EnvironmentPtr envPtr, OSInstance *instance);
    virtual ~NLPSolverIpoptRelaxed();

    virtual VectorDouble getSolution();

  protected:
    virtual void setSolverSpecificInitialSettings();

  private:
};
} // namespace SHOT