/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "INLPSolver.h"
#include "NLPSolverBase.h"

#include "OSIpoptSolver.h"
#include "OSOption.h"

class NLPSolverIpoptBase : virtual public INLPSolver
{
  private:
  protected:
    OSOption *osOption;
    IpoptSolver *IpoptNLPSolver;
    OSoLWriter *osolwriter;

    std::vector<int> fixedVariableIndexes;
    DoubleVector fixedVariableValues;

    std::vector<int> startingPointVariableIndexes;
    DoubleVector startingPointVariableValues;

    virtual E_NLPSolutionStatus solveProblemInstance();

    void fixVariables(std::vector<int> variableIndexes, DoubleVector variableValues);
    void unfixVariables();

    virtual void setInitialSettings();
    virtual void setSolverSpecificInitialSettings() = 0;
    virtual void updateSettings();

    virtual DoubleVector getCurrentVariableLowerBounds();
    virtual DoubleVector getCurrentVariableUpperBounds();

    DoubleVector lowerBoundsBeforeFix;
    DoubleVector upperBoundsBeforeFix;

  public:
    NLPSolverIpoptBase();
    virtual ~NLPSolverIpoptBase();

    virtual void setStartingPoint(std::vector<int> variableIndexes, DoubleVector variableValues);
    virtual void clearStartingPoint();

    virtual DoubleVector getSolution();
    virtual double getSolution(int i);
    virtual double getObjectiveValue();

    virtual bool isObjectiveFunctionNonlinear();
    virtual int getObjectiveFunctionVariableIndex();

    virtual void saveOptionsToFile(std::string fileName);
};
