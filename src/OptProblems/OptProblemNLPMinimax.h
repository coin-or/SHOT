/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "OptProblem.h"

class OptProblemNLPMinimax : public OptProblem
{
  public:
    OptProblemNLPMinimax();
    virtual ~OptProblemNLPMinimax();

    void reformulate(OSInstance *originalProblem);

    virtual void copyVariables(OSInstance *source, OSInstance *destination, bool integerRelaxed);
    virtual void copyObjectiveFunction(OSInstance *source, OSInstance *destination);
    virtual void copyConstraints(OSInstance *source, OSInstance *destination);
    virtual void copyLinearTerms(OSInstance *source, OSInstance *destination);
    virtual void copyQuadraticTerms(OSInstance *source, OSInstance *destination);
    virtual void copyNonlinearExpressions(OSInstance *source, OSInstance *destination);

  private:
    int muindex;
    OSiLReader *osilReader = NULL;
};
