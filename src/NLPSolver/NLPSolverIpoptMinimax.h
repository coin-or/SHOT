/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "NLPSolverIpoptBase.h"
#include "../OptProblems/OptProblemNLPMinimax.h"

namespace SHOT
{
class NLPSolverIpoptMinimax : public NLPSolverBase, public NLPSolverIpoptBase
{
  public:
    NLPSolverIpoptMinimax(EnvironmentPtr envPtr);
    virtual ~NLPSolverIpoptMinimax();

    VectorDouble getSolution();

  protected:
    bool createProblemInstance(OSInstance *origInstance);

    void setSolverSpecificInitialSettings();

  private:
};
} // namespace SHOT