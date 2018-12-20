/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "PrimalSolutionStrategyBase.h"

#include "../NLPSolver/NLPSolverIpoptRelaxed.h"

#include "../MIPSolver/IMIPSolver.h"

#ifdef HAS_GAMS
#include "../NLPSolver/NLPSolverGAMS.h"
#include "../ModelingSystem/ModelingSystemGAMS.h"
#endif

#include "../Tasks/TaskSelectHyperplanePointsESH.h"
#include "../Tasks/TaskSelectHyperplanePointsECP.h"

#include "../ModelingSystem/ModelingSystemOS.h"

namespace SHOT
{
class PrimalSolutionStrategyFixedNLP : public PrimalSolutionStrategyBase
{
  public:
    PrimalSolutionStrategyFixedNLP(EnvironmentPtr envPtr);
    virtual ~PrimalSolutionStrategyFixedNLP();

    virtual bool runStrategy();

  protected:
  private:
    std::shared_ptr<INLPSolver> NLPSolver;

    VectorInteger discreteVariableIndexes;
    std::vector<VectorDouble> testedPoints;
    VectorDouble fixPoint;

    double originalNLPTime;
    double originalNLPIter;

    VectorDouble originalLBs;
    VectorDouble originalUBs;

    TaskBase *taskSelectHPPts;

    int originalIterFrequency;
    double originalTimeFrequency;
};
} // namespace SHOT