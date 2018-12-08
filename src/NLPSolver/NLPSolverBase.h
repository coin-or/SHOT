/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "INLPSolver.h"

namespace SHOT
{
class NLPSolverBase : virtual public INLPSolver
{
  private:
  protected:
    NLPSolverBase();

    //bool isProblemInitialized = false;

  public:
    //virtual void setProblem(OSInstance *origInstance);
    //virtual void initializeProblem();
    virtual E_NLPSolutionStatus solveProblem();

    //virtual void saveProblemToFile(std::string fileName);

    //virtual VectorDouble getCurrentVariableLowerBounds() = 0;
    //virtual VectorDouble getCurrentVariableUpperBounds() = 0;
};
} // namespace SHOT