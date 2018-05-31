/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "IRelaxationStrategy.h"
#include "RelaxationStrategyBase.h"

class RelaxationStrategyNone : public IRelaxationStrategy, public RelaxationStrategyBase
{
  public:
    RelaxationStrategyNone(IMIPSolver *MIPSolver);
    virtual ~RelaxationStrategyNone();

    virtual void executeStrategy();

    virtual void setActive();

    virtual void setInactive();

    virtual void setInitial();

    virtual E_IterationProblemType getProblemType();

  private:
    IMIPSolver *MIPSolver;
};
