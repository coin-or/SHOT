/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "PrimalSolutionStrategyBase.h"
#include "../NLPSolver/INLPSolver.h"
#include "../NLPSolver/NLPSolverIpoptRelaxed.h"
#include "../NLPSolver/NLPSolverCuttingPlaneRelaxed.h"
#include "../Report.h"

#ifdef HAS_GAMS
#include "../NLPSolver/NLPSolverGAMS.h"
#endif

#include "../Tasks/TaskSelectHyperplanePointsLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsIndividualLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsSolution.h"

class PrimalSolutionStrategyFixedNLP : public PrimalSolutionStrategyBase
{
  public:
    PrimalSolutionStrategyFixedNLP(EnvironmentPtr envPtr);
    virtual ~PrimalSolutionStrategyFixedNLP();

    virtual bool runStrategy();

  protected:
  private:
    INLPSolver *NLPSolver;

    std::vector<int> discreteVariableIndexes;
    std::vector<std::vector<double>> testedPoints;
    std::vector<double> fixPoint;

    double originalNLPTime;
    double originalNLPIter;

    std::vector<double> originalLBs;
    std::vector<double> originalUBs;

    TaskBase *taskSelectHPPts;

    int originalIterFrequency;
    double originalTimeFrequency;
};
