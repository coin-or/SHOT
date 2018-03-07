/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "SHOTSettings.h"
#include "../ProcessInfo.h"
#include "INLPSolver.h"
#include "iterator"
#include "vector"

class NLPSolverBase : virtual public INLPSolver
{
  private:
  protected:
    OSInstance *originalInstance;

    bool isProblemInitialized;

  public:
    virtual void setProblem(OSInstance *origInstance);
    virtual void initializeProblem();
    virtual E_NLPSolutionStatus solveProblem();

    virtual void saveProblemToFile(std::string fileName);

    virtual std::vector<double> getVariableLowerBounds();
    virtual std::vector<double> getVariableUpperBounds();
};
