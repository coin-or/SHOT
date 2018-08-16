/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "OptProblemOriginal.h"

class OptProblemOriginalNonlinearObjective : public OptProblemOriginal
{
  public:
    OptProblemOriginalNonlinearObjective(EnvironmentPtr envPtr);
    ~OptProblemOriginalNonlinearObjective();

    virtual bool setProblem(OSInstance *instance);
    double calculateConstraintFunctionValue(int idx, DoubleVector point);
    SparseVector *calculateConstraintFunctionGradient(int idx, DoubleVector point);
    virtual int getNumberOfNonlinearConstraints();
    virtual int getNumberOfConstraints();
    virtual std::vector<std::string> getConstraintNames();
    virtual int getNumberOfVariables();
    virtual int getNumberOfRealVariables();
    virtual std::vector<std::string> getVariableNames();
    virtual std::vector<char> getVariableTypes();

    virtual DoubleVector getVariableLowerBounds();
    virtual DoubleVector getVariableUpperBounds();

    virtual double getVariableLowerBound(int varIdx);
    virtual double getVariableUpperBound(int varIdx);

    virtual void setVariableUpperBound(int varIdx, double value);
    virtual void setVariableLowerBound(int varIdx, double value);

    virtual std::vector<std::pair<int, double>> getObjectiveFunctionVarCoeffPairs();
    virtual double getObjectiveConstant();

    IndexValuePair getMostDeviatingAllConstraint(DoubleVector point);

    virtual void setNonlinearConstraintIndexes();

  private:
    std::string addedObjectiveVariableName;
    std::string addedConstraintName;
    double addedObjectiveVariableLowerBound;
    double addedObjectiveVariableUpperBound;
};
