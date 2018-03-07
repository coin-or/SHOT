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
    IpoptSolver *NLPSolver;
    OSoLWriter *osolwriter;

    std::vector<int> fixedVariableIndexes;
    std::vector<double> fixedVariableValues;

    std::vector<int> startingPointVariableIndexes;
    std::vector<double> startingPointVariableValues;

    virtual E_NLPSolutionStatus solveProblemInstance();

    void fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues);
    void unfixVariables();

    virtual void setInitialSettings();
    virtual void setSolverSpecificInitialSettings() = 0;
    virtual void updateSettings();

    virtual std::vector<double> getCurrentVariableLowerBounds();
    virtual std::vector<double> getCurrentVariableUpperBounds();

    std::vector<double> lowerBoundsBeforeFix;
    std::vector<double> upperBoundsBeforeFix;

  public:
    NLPSolverIpoptBase();
    ~NLPSolverIpoptBase();

    virtual void setStartingPoint(std::vector<int> variableIndexes, std::vector<double> variableValues);
    virtual void clearStartingPoint();

    virtual std::vector<double> getSolution();
    virtual double getSolution(int i);
    virtual double getObjectiveValue();

    virtual bool isObjectiveFunctionNonlinear();
    virtual int getObjectiveFunctionVariableIndex();

    virtual void saveOptionsToFile(std::string fileName);
};
