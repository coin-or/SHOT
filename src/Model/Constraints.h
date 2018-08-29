
/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "ModelStructs.h"
#include "Terms.h"
#include "NonlinearExpressions.h"
#include <vector>
#include <string>
#include <memory>
#include <limits>
#include <algorithm>

namespace SHOT
{

enum class E_ConstraintClassification
{
    None,
    Linear,
    Quadratic,
    QuadraticConsideredAsNonlinear,
    Signomial,
    Nonlinear,
    GeneralizedSignomial,
    Nonalgebraic
};

enum class E_ConstraintSignType
{
    None,
    Equality,
    LessThan,
    GreaterThan,
    LessThanAndGreaterThan
};

struct ConstraintProperties
{
    E_ConstraintClassification classification;
    E_Curvature curvature;
    E_ConstraintSignType type;

    bool isReformulated = false;

    bool hasLinearTerms = false;
    bool hasSignomialTerms = false;
    bool hasQuadraticTerms = false;
    bool hasNonlinearExpression = false;
    bool hasNonalgebraicPart = false; // E.g. for external functions
};

class Constraint
{
  public:
    virtual ~Constraint(){};

    int constraintIndex;
    std::string constraintName;

    ConstraintProperties properties;
    virtual void updateProperties();

    virtual bool isConstraintValid(VectorDouble point) = 0;
};

typedef std::shared_ptr<Constraint> ConstraintPtr;
typedef std::vector<ConstraintPtr> Constraints;

class NumericConstraint;

typedef std::shared_ptr<NumericConstraint> NumericConstraintPtr;
typedef std::vector<NumericConstraintPtr> NumericConstraints;

struct NumericConstraintValue
{
    NumericConstraintPtr constraint;
    double functionValue;

    bool isFulfilledLHS;
    double errorLHS;

    bool isFulfilledRHS;
    double errorRHS;

    bool isFulfilled;
    double error;
};

typedef std::vector<NumericConstraintValue> NumericConstraintValues;

class NumericConstraint : std::enable_shared_from_this<NumericConstraint>, public Constraint
{
  public:
    double valueLHS = -std::numeric_limits<double>::infinity();
    double valueRHS = std::numeric_limits<double>::infinity();

    virtual double calculateFunctionValue(const VectorDouble &point) = 0;

    virtual NumericConstraintValue calculateNumericValue(const VectorDouble &point)
    {
        double value = calculateFunctionValue(point);

        NumericConstraintValue constrValue;
        constrValue.constraint = getPointer();

        constrValue.isFulfilledRHS = (value <= valueRHS);
        constrValue.errorRHS = std::max(value - valueRHS, 0.0);

        constrValue.isFulfilledLHS = (value >= valueLHS);
        constrValue.errorLHS = std::max(valueLHS - value, 0.0);

        constrValue.isFulfilled = (constrValue.isFulfilledRHS && constrValue.isFulfilledLHS);
        constrValue.error = std::max(constrValue.errorRHS, constrValue.errorLHS);

        return constrValue;
    }

    virtual bool isConstraintValid(const VectorDouble &point)
    {
        auto constraintValue = calculateNumericValue(point);

        return (constraintValue.isFulfilledLHS && constraintValue.isFulfilledRHS);
    };

    virtual std::shared_ptr<NumericConstraint> getPointer()
    {
        return shared_from_this();
    }
};

class LinearConstraint : public NumericConstraint
{
  public:
    LinearConstraint(){};

    LinearTerms linearTerms;

    virtual double calculateFunctionValue(const VectorDouble &point)
    {
        double value = linearTerms.calculate(point);
    }
};

typedef std::shared_ptr<LinearConstraint> LinearConstraintPtr;
typedef std::vector<LinearConstraintPtr> LinearConstraints;

class QuadraticConstraint : public NumericConstraint
{
  public:
    QuadraticConstraint()
    {
    }

    LinearTerms linearTerms;
    QuadraticTerms quadraticTerms;

    virtual double calculateFunctionValue(const VectorDouble &point)
    {
        double value = linearTerms.calculate(point);
        value += quadraticTerms.calculate(point);

        return value;
    }
};

typedef std::shared_ptr<QuadraticConstraint> QuadraticConstraintPtr;
typedef std::vector<QuadraticConstraintPtr> QuadraticConstraints;

class NonlinearConstraint : public NumericConstraint
{
  public:
    NonlinearConstraint();

    LinearTerms linearTerms;
    QuadraticTerms quadraticTerms;
    NonlinearExpressionPtr nonlinearExpression;

    virtual double calculateFunctionValue(const VectorDouble &point)
    {
        double value = linearTerms.calculate(point);
        value += quadraticTerms.calculate(point);
        value += nonlinearExpression->calculate(point);

        return value;
    }
};

typedef std::shared_ptr<NonlinearConstraint> NonlinearConstraintPtr;
typedef std::vector<NonlinearConstraintPtr> NonlinearConstraints;
} // namespace SHOT