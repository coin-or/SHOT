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

namespace SHOT
{
class NLPSolverIpoptBase : virtual public INLPSolver
{
  private:
  protected:
    OSOption *osOption;
    IpoptSolver *IpoptNLPSolver;
    OSoLWriter *osolwriter;

    VectorInteger fixedVariableIndexes;
    VectorDouble fixedVariableValues;

    VectorInteger startingPointVariableIndexes;
    VectorDouble startingPointVariableValues;

    virtual E_NLPSolutionStatus solveProblemInstance();

    void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues);
    void unfixVariables();

    virtual void setInitialSettings();
    virtual void setSolverSpecificInitialSettings() = 0;
    virtual void updateSettings();

    virtual VectorDouble getCurrentVariableLowerBounds();
    virtual VectorDouble getCurrentVariableUpperBounds();

    VectorDouble lowerBoundsBeforeFix;
    VectorDouble upperBoundsBeforeFix;

  public:
    NLPSolverIpoptBase();
    virtual ~NLPSolverIpoptBase();

    virtual void setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues);
    virtual void clearStartingPoint();

    virtual VectorDouble getSolution();
    virtual double getSolution(int i);
    virtual double getObjectiveValue();

    virtual bool isObjectiveFunctionNonlinear();
    virtual int getObjectiveFunctionVariableIndex();

    virtual void saveOptionsToFile(std::string fileName);
};
} // namespace SHOT