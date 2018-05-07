/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"
#include "CoinFinite.hpp"
#include "SHOTSettings.h"
#include "../ProcessInfo.h"
#include "../UtilityFunctions.h"

class OptProblem
{
  public:
    OptProblem();
    virtual ~OptProblem();

    void setNonlinearObjectiveConstraintIdx(int idx);
    int getNonlinearObjectiveConstraintIdx();

    void setNonlinearObjectiveVariableIdx(int idx);
    int getNonlinearObjectiveVariableIdx();

    void setObjectiveFunctionType(E_ObjectiveFunctionType type);
    E_ObjectiveFunctionType getObjectiveFunctionType();

    virtual int getNumberOfNonlinearConstraints();
    virtual int getNumberOfQuadraticConstraints();
    virtual int getNumberOfNonlinearConstraints(OSInstance *instance);
    virtual int getNumberOfLinearConstraints();
    virtual int getNumberOfConstraints();
    virtual std::vector<std::string> getConstraintNames();

    virtual int getNumberOfVariables();
    virtual int getNumberOfBinaryVariables();
    virtual int getNumberOfDiscreteVariables();
    virtual int getNumberOfIntegerVariables();
    virtual int getNumberOfRealVariables();
    virtual std::vector<std::string> getVariableNames();
    virtual std::vector<char> getVariableTypes();
    virtual std::vector<double> getVariableLowerBounds();
    virtual std::vector<double> getVariableUpperBounds();

    virtual double getVariableLowerBound(int varIdx);
    virtual double getVariableUpperBound(int varIdx);

    virtual void setVariableUpperBound(int varIdx, double value);
    virtual void setVariableLowerBound(int varIdx, double value);

    virtual std::vector<int> getRealVariableIndices();
    virtual std::vector<int> getDiscreteVariableIndices();
    virtual std::vector<int> getBinaryVariableIndices();
    virtual std::vector<int> getIntegerVariableIndices();

    virtual std::vector<std::pair<int, double>> getObjectiveFunctionVarCoeffPairs();
    virtual std::vector<QuadraticTerm *> getQuadraticTermsInConstraint(int constrIdx);
    virtual double getObjectiveConstant();

    void exportProblemToOsil(std::string fileName);

    void saveProblemModelToFile(std::string fileName);

    std::string exportProblemToOsil();

    virtual IndexValuePair getMostDeviatingConstraint(std::vector<double> point);

    virtual std::pair<IndexValuePair, std::vector<int>> getMostDeviatingConstraint(std::vector<double> point,
                                                                                   std::vector<int> constrIdxs);

    virtual IndexValuePair getMostDeviatingAllConstraint(std::vector<double> point);

    virtual std::vector<IndexValuePair> getMostDeviatingConstraints(std::vector<double> point, double tolerance);

    virtual bool isConstraintsFulfilledInPoint(std::vector<double> point);
    virtual bool isConstraintsFulfilledInPoint(std::vector<double> point, double eps);

    virtual bool isDiscreteVariablesFulfilledInPoint(std::vector<double> point, double eps);
    virtual bool isVariableBoundsFulfilledInPoint(std::vector<double> point, double eps);

    virtual bool isLinearConstraintsFulfilledInPoint(std::vector<double> point);
    virtual bool isLinearConstraintsFulfilledInPoint(std::vector<double> point, double eps);

    virtual SparseVector *calculateConstraintFunctionGradient(int idx, std::vector<double> point);

    virtual double calculateConstraintFunctionValue(int idx, std::vector<double> point);
    virtual double calculateOriginalObjectiveValue(std::vector<double> point);

    std::vector<int> getNonlinearConstraintIndexes();
    void setNonlinearConstraints(std::vector<int> idxs);

    std::vector<int> getLinearConstraintIndexes();

    std::vector<int> getQuadraticConstraintIndexes();
    void setQuadraticConstraints(std::vector<int> idxs);

    std::vector<int> getNonlinearOrQuadraticConstraintIndexes();

    virtual bool isTypeOfObjectiveMinimize();
    virtual bool isObjectiveFunctionNonlinear();

    void setTypeOfObjectiveMinimize(bool value);
    void setObjectiveFunctionNonlinear(bool value);

    bool isConstraintNonlinear(int constrIdx);
    bool isConstraintQuadratic(int constrIdx);

    bool isConstraintNonlinear(OSInstance *instance, int idx);
    bool isProblemNonlinear(OSInstance *instance);

    bool isProblemDiscrete();

    bool hasVariableBoundsBeenTightened(int varIndex);
    void setVariableBoundsAsTightened(int varIndex);
    void setVariableBoundsTightened(std::vector<bool> status);

    OSInstance *getProblemInstance();

    std::vector<double> calculateGradientNumerically(int constraintIndex, std::vector<double> point);

    virtual void fixVariable(int varIdx, double value);

  protected:
    void setNonlinearConstraintIndexes();

    virtual void copyVariables(OSInstance *source, OSInstance *destination, bool integerRelaxed);
    virtual void copyObjectiveFunction(OSInstance *source, OSInstance *destination);
    virtual void copyQuadraticTerms(OSInstance *source, OSInstance *destination);
    virtual void copyNonlinearExpressions(OSInstance *source, OSInstance *destination);
    virtual void copyLinearTerms(OSInstance *source, OSInstance *destination);
    virtual void copyConstraints(OSInstance *source, OSInstance *destination);

    void repairNonboundedVariables();

    void setProblemInstance(OSInstance *instance);

  private:
    std::vector<int> m_nonlinearConstraints;
    std::vector<int> m_quadraticConstraints;
    std::vector<int> m_nonlinearOrQuadraticConstraints;
    std::vector<bool> m_variableBoundTightened;

    bool m_isTypeOfObjectiveMinimize;

    bool m_isObjectiveFunctionNonlinear;

    OSInstance *m_problemInstance;

    int m_idxNonlinearObjectiveConstraint;
    int m_idxNonlinearObjectiveVariable;

    E_ObjectiveFunctionType m_objectiveFunctionType;
};
