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
#include "../Enums.h"
#include "../Structs.h"
#include "../Environment.h"
#include "../Model.h"

#include "SHOTSettings.h"
#include "../UtilityFunctions.h"

namespace SHOT
{
class OptProblem
{
  public:
    OptProblem(EnvironmentPtr envPtr);
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
    virtual VectorString getConstraintNames();

    virtual int getNumberOfVariables();
    virtual int getNumberOfBinaryVariables();
    virtual int getNumberOfDiscreteVariables();
    virtual int getNumberOfIntegerVariables();
    virtual int getNumberOfRealVariables();
    virtual VectorString getVariableNames();
    virtual std::vector<char> getVariableTypes();
    virtual VectorDouble getVariableLowerBounds();
    virtual VectorDouble getVariableUpperBounds();

    virtual double getVariableLowerBound(int varIdx);
    virtual double getVariableUpperBound(int varIdx);

    virtual void setVariableUpperBound(int varIdx, double value);
    virtual void setVariableLowerBound(int varIdx, double value);

    virtual VectorInteger getRealVariableIndices();
    virtual VectorInteger getDiscreteVariableIndices();
    virtual VectorInteger getBinaryVariableIndices();
    virtual VectorInteger getIntegerVariableIndices();

    virtual std::vector<std::pair<int, double>> getObjectiveFunctionVarCoeffPairs();
    virtual std::vector<QuadraticTerm *> getQuadraticTermsInConstraint(int constrIdx);
    virtual double getObjectiveConstant();

    void exportProblemToOsil(std::string fileName);

    void saveProblemModelToFile(std::string fileName);

    std::string exportProblemToOsil();

    virtual PairIndexValue getMostDeviatingConstraint(VectorDouble point);

    virtual std::pair<PairIndexValue, VectorInteger> getMostDeviatingConstraint(VectorDouble point,
                                                                                VectorInteger constrIdxs);

    virtual PairIndexValue getMostDeviatingAllConstraint(VectorDouble point);

    virtual std::vector<PairIndexValue> getMostDeviatingConstraints(VectorDouble point, double tolerance);

    virtual bool isConstraintsFulfilledInPoint(VectorDouble point);
    virtual bool isConstraintsFulfilledInPoint(VectorDouble point, double eps);

    virtual bool isDiscreteVariablesFulfilledInPoint(VectorDouble point, double eps);
    virtual bool isVariableBoundsFulfilledInPoint(VectorDouble point, double eps);

    virtual bool isLinearConstraintsFulfilledInPoint(VectorDouble point);
    virtual bool isLinearConstraintsFulfilledInPoint(VectorDouble point, double eps);

    virtual SparseVector *calculateConstraintFunctionGradient(int idx, VectorDouble point);

    virtual double calculateConstraintFunctionValue(int idx, VectorDouble point);
    virtual double calculateOriginalObjectiveValue(VectorDouble point);

    VectorInteger getNonlinearConstraintIndexes();
    void setNonlinearConstraints(VectorInteger idxs);

    VectorInteger getLinearConstraintIndexes();

    VectorInteger getQuadraticConstraintIndexes();
    void setQuadraticConstraints(VectorInteger idxs);

    VectorInteger getNonlinearOrQuadraticConstraintIndexes();

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

    VectorDouble calculateGradientNumerically(int constraintIndex, VectorDouble point);

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

    EnvironmentPtr env;

  private:
    VectorInteger m_nonlinearConstraints;
    VectorInteger m_quadraticConstraints;
    VectorInteger m_nonlinearOrQuadraticConstraints;
    std::vector<bool> m_variableBoundTightened;

    bool m_isTypeOfObjectiveMinimize;

    bool m_isObjectiveFunctionNonlinear;

    OSInstance *m_problemInstance;

    int m_idxNonlinearObjectiveConstraint;
    int m_idxNonlinearObjectiveVariable;

    E_ObjectiveFunctionType m_objectiveFunctionType;
};
} // namespace SHOT