/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "NLPSolverBase.h"
#include "../Report.h"
#include "../OptProblems/OptProblemNLPMinimax.h"

#ifdef HAS_CPLEX
#include "../MIPSolver/MIPSolverCplex.h"
#endif
#ifdef HAS_GUROBI
#include "../MIPSolver/MIPSolverGurobi.h"
#endif

#include "../MIPSolver/MIPSolverOsiCbc.h"

#include "boost/math/tools/minima.hpp"

class NLPSolverCuttingPlaneMinimax : public NLPSolverBase
{
  public:
    NLPSolverCuttingPlaneMinimax(EnvironmentPtr envPtr);
    virtual ~NLPSolverCuttingPlaneMinimax();

    virtual void setStartingPoint(std::vector<int> variableIndexes, DoubleVector variableValues);
    virtual void clearStartingPoint();

    virtual bool isObjectiveFunctionNonlinear();
    virtual int getObjectiveFunctionVariableIndex();

    virtual DoubleVector getCurrentVariableLowerBounds();
    virtual DoubleVector getCurrentVariableUpperBounds();

  private:
    IMIPSolver *LPSolver;

    virtual double getSolution(int i);
    virtual DoubleVector getSolution();
    virtual double getObjectiveValue();

    virtual bool createProblemInstance(OSInstance *origInstance);

    virtual void fixVariables(std::vector<int> variableIndexes, DoubleVector variableValues);

    virtual void unfixVariables();

    virtual E_NLPSolutionStatus solveProblemInstance();

    virtual void saveOptionsToFile(std::string fileName);

    bool isProblemCreated;

    DoubleVector solution;
    double objectiveValue = NAN;
};
