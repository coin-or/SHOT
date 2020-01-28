/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include <string>
#include <memory>

#include "../Enums.h"
#include "../Structs.h"

#include "Variables.h"
#include "Terms.h"
#include "NonlinearExpressions.h"

#include "cppad/cppad.hpp"
#include "cppad/utility.hpp"

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
    E_ConstraintClassification classification = E_ConstraintClassification::None;
    E_Convexity convexity = E_Convexity::NotSet;
    E_ConstraintSignType type = E_ConstraintSignType::None;
    E_Monotonicity monotonicity = E_Monotonicity::NotSet;

    bool isReformulated = false;

    bool hasLinearTerms = false;
    bool hasQuadraticTerms = false;
    bool hasMonomialTerms = false;
    bool hasSignomialTerms = false;
    bool hasNonlinearExpression = false;
    bool hasNonalgebraicPart = false; // E.g. for external functions
};

class Constraint
{
public:
    virtual ~Constraint() = default;

    int index = -1;
    std::string name;

    ConstraintProperties properties;

    std::weak_ptr<Problem> ownerProblem;

    virtual bool isFulfilled(const VectorDouble& point) = 0;

    virtual void takeOwnership(ProblemPtr owner) = 0;

    virtual std::ostream& print(std::ostream&) const = 0;

    virtual void updateProperties() = 0;
};

using ConstraintPtr = std::shared_ptr<Constraint>;

std::ostream& operator<<(std::ostream& stream, ConstraintPtr constraint);
std::ostream& operator<<(std::ostream& stream, const Constraint& constraint);

class NumericConstraint;
using NumericConstraintPtr = std::shared_ptr<NumericConstraint>;
using NumericConstraints = std::vector<NumericConstraintPtr>;

struct NumericConstraintValue
{
    NumericConstraintPtr constraint;

    // Considering a constraint L <= f(x) <= U:
    double functionValue; // This is the function value of f(x)
    bool isFulfilledLHS; // Is L <= f(x)?
    double normalizedLHSValue; // This is the value of L - f(x)
    bool isFulfilledRHS; // Is f(x) <= U
    double normalizedRHSValue; // This is the value of f(x) - U
    bool isFulfilled; // Is L <= f(x) <= U?
    double error; // max(0, max(L - f(x), f(x) - U)
    double normalizedValue; // max(L - f(x), f(x)-U)

    // Sorts in reverse order, i.e. so that larger errors are before smaller ones
    bool operator>(const NumericConstraintValue& otherValue) const
    {
        return normalizedValue > otherValue.normalizedValue;
    }
};

using NumericConstraintValues = std::vector<NumericConstraintValue>;

class NumericConstraint : public Constraint, public std::enable_shared_from_this<NumericConstraint>
{
public:
    double valueLHS = SHOT_DBL_MIN;
    double valueRHS = SHOT_DBL_MAX;
    double constant = 0.0;

    std::shared_ptr<Variables> gradientSparsityPattern;
    std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> hessianSparsityPattern;

    virtual double calculateFunctionValue(const VectorDouble& point) = 0;
    virtual Interval calculateFunctionValue(const IntervalVector& intervalVector) = 0;

    virtual Interval getConstraintFunctionBounds() = 0;

    virtual SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) = 0;
    virtual std::shared_ptr<Variables> getGradientSparsityPattern();

    // Returns the upper triagonal part of the Hessian matrix is sparse representation
    virtual SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) = 0;
    virtual std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> getHessianSparsityPattern();

    virtual NumericConstraintValue calculateNumericValue(const VectorDouble& point, double correction = 0.0);

    bool isFulfilled(const VectorDouble& point) override;

    void takeOwnership(ProblemPtr owner) override = 0;

    virtual std::shared_ptr<NumericConstraint> getPointer() = 0;

    void updateProperties() override = 0;

protected:
    virtual void initializeGradientSparsityPattern() = 0;
    virtual void initializeHessianSparsityPattern() = 0;
};

class LinearConstraint : public NumericConstraint
{
public:
    LinearTerms linearTerms;

    LinearConstraint() = default;

    LinearConstraint(int constraintIndex, std::string constraintName, double LHS, double RHS)
    {
        index = constraintIndex;
        name = constraintName;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    LinearConstraint(int constraintIndex, std::string constraintName, LinearTerms linTerms, double LHS, double RHS)
    {
        index = constraintIndex;
        name = constraintName;
        linearTerms = linTerms;
        valueLHS = LHS;
        valueRHS = RHS;

        properties.hasLinearTerms = linearTerms.size() > 0 ? true : false;
    };

    void add(LinearTerms terms);
    void add(LinearTermPtr term);

    double calculateFunctionValue(const VectorDouble& point) override;
    Interval calculateFunctionValue(const IntervalVector& intervalVector) override;

    Interval getConstraintFunctionBounds() override;

    bool isFulfilled(const VectorDouble& point) override;

    void takeOwnership(ProblemPtr owner) override;

    SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) override;

    // Returns the upper triagonal part of the Hessian matrix is sparse representation
    SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) override;

    NumericConstraintValue calculateNumericValue(const VectorDouble& point, double correction = 0.0) override;

    std::shared_ptr<NumericConstraint> getPointer() override;

    void updateProperties() override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    void initializeGradientSparsityPattern() override;
    void initializeHessianSparsityPattern() override;
};

using LinearConstraintPtr = std::shared_ptr<LinearConstraint>;
using LinearConstraints = std::vector<LinearConstraintPtr>;

std::ostream& operator<<(std::ostream& stream, LinearConstraintPtr constraint);

class QuadraticConstraint : public LinearConstraint
{
public:
    QuadraticTerms quadraticTerms;

    QuadraticConstraint() : LinearConstraint() {};

    QuadraticConstraint(int constraintIndex, std::string constraintName, double LHS, double RHS)
    {
        index = constraintIndex;
        name = constraintName;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    QuadraticConstraint(
        int constraintIndex, std::string constraintName, QuadraticTerms quadTerms, double LHS, double RHS)
    {
        index = constraintIndex;
        name = constraintName;
        quadraticTerms = quadTerms;
        valueLHS = LHS;
        valueRHS = RHS;

        properties.hasQuadraticTerms = quadraticTerms.size() > 0 ? true : false;
    };

    QuadraticConstraint(int constraintIndex, std::string constraintName, LinearTerms linTerms, QuadraticTerms quadTerms,
        double LHS, double RHS)
    {
        index = constraintIndex;
        name = constraintName;
        linearTerms = linTerms;
        quadraticTerms = quadTerms;
        valueLHS = LHS;
        valueRHS = RHS;

        properties.hasLinearTerms = linearTerms.size() > 0 ? true : false;
        properties.hasQuadraticTerms = quadraticTerms.size() > 0 ? true : false;
    };

    void add(LinearTerms terms);
    void add(LinearTermPtr term);
    void add(QuadraticTerms terms);
    void add(QuadraticTermPtr term);

    double calculateFunctionValue(const VectorDouble& point) override;
    Interval calculateFunctionValue(const IntervalVector& intervalVector) override;

    Interval getConstraintFunctionBounds() override;

    bool isFulfilled(const VectorDouble& point) override;

    void takeOwnership(ProblemPtr owner) override;

    SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) override;

    // Returns the upper triagonal part of the Hessian matrix is sparse representation
    SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) override;

    NumericConstraintValue calculateNumericValue(const VectorDouble& point, double correction = 0.0) override;

    std::shared_ptr<NumericConstraint> getPointer() override;

    void updateProperties() override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    void initializeGradientSparsityPattern() override;
    void initializeHessianSparsityPattern() override;
};

using QuadraticConstraintPtr = std::shared_ptr<QuadraticConstraint>;
using QuadraticConstraints = std::vector<QuadraticConstraintPtr>;

std::ostream& operator<<(std::ostream& stream, QuadraticConstraintPtr constraint);

class NonlinearConstraint : public QuadraticConstraint
{
public:
    MonomialTerms monomialTerms;
    SignomialTerms signomialTerms;

    NonlinearExpressionPtr nonlinearExpression;
    FactorableFunctionPtr factorableFunction;

    CppAD::sparse_rc<std::vector<size_t>> nonlinearGradientSparsityPattern;
    CppAD::sparse_rc<std::vector<size_t>> nonlinearHessianSparsityPattern;

    bool nonlinearGradientSparsityMapGenerated = false;
    bool nonlinearHessianSparsityMapGenerated = false;

    Variables variablesInMonomialTerms;
    Variables variablesInSignomialTerms;
    Variables variablesInNonlinearExpression;

    int nonlinearExpressionIndex = -1;

    NonlinearConstraint() = default;

    NonlinearConstraint(int constraintIndex, std::string constraintName, double LHS, double RHS)
    {
        index = constraintIndex;
        name = constraintName;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    NonlinearConstraint(
        int constraintIndex, std::string constraintName, NonlinearExpressionPtr expression, double LHS, double RHS)
    {
        index = constraintIndex;
        name = constraintName;
        nonlinearExpression = expression;
        valueLHS = LHS;
        valueRHS = RHS;

        properties.hasNonlinearExpression = true;
    };

    NonlinearConstraint(int constraintIndex, std::string constraintName, QuadraticTerms quadTerms,
        NonlinearExpressionPtr expression, double LHS, double RHS)
    {
        index = constraintIndex;
        name = constraintName;
        quadraticTerms = quadTerms;
        nonlinearExpression = expression;
        valueLHS = LHS;
        valueRHS = RHS;

        properties.hasQuadraticTerms = quadraticTerms.size() > 0 ? true : false;
        properties.hasNonlinearExpression = true;
    };

    NonlinearConstraint(int constraintIndex, std::string constraintName, LinearTerms linTerms,
        NonlinearExpressionPtr expression, double LHS, double RHS)
    {
        index = constraintIndex;
        name = constraintName;
        linearTerms = linTerms;
        nonlinearExpression = expression;
        valueLHS = LHS;
        valueRHS = RHS;

        properties.hasLinearTerms = linearTerms.size() > 0 ? true : false;
        properties.hasNonlinearExpression = true;
    };

    NonlinearConstraint(int constraintIndex, std::string constraintName, LinearTerms linTerms, QuadraticTerms quadTerms,
        NonlinearExpressionPtr expression, double LHS, double RHS)
    {
        index = constraintIndex;
        name = constraintName;
        linearTerms = linTerms;
        quadraticTerms = quadTerms;
        nonlinearExpression = expression;
        valueLHS = LHS;
        valueRHS = RHS;

        properties.hasLinearTerms = linearTerms.size() > 0 ? true : false;
        properties.hasQuadraticTerms = quadraticTerms.size() > 0 ? true : false;
        properties.hasNonlinearExpression = true;
    };

    void add(LinearTerms terms);
    void add(LinearTermPtr term);
    void add(QuadraticTerms terms);
    void add(QuadraticTermPtr term);
    void add(MonomialTerms terms);
    void add(MonomialTermPtr term);
    void add(SignomialTerms terms);
    void add(SignomialTermPtr term);
    void add(NonlinearExpressionPtr expression);

    void updateFactorableFunction();

    double calculateFunctionValue(const VectorDouble& point) override;

    Interval getConstraintFunctionBounds() override;

    SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) override;

    // Returns the upper triagonal part of the Hessian matrix is sparse representation
    SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) override;

    Interval calculateFunctionValue(const IntervalVector& intervalVector) override;

    bool isFulfilled(const VectorDouble& point) override;

    void takeOwnership(ProblemPtr owner) override;

    NumericConstraintValue calculateNumericValue(const VectorDouble& point, double correction = 0.0) override;

    std::shared_ptr<NumericConstraint> getPointer() override;

    void updateProperties() override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    void initializeGradientSparsityPattern() override;
    void initializeHessianSparsityPattern() override;
};

using NonlinearConstraintPtr = std::shared_ptr<NonlinearConstraint>;
using NonlinearConstraints = std::vector<NonlinearConstraintPtr>;

std::ostream& operator<<(std::ostream& stream, NonlinearConstraintPtr constraint);
std::ostream& operator<<(std::ostream& stream, NumericConstraintPtr constraint);
std::ostream& operator<<(std::ostream& stream, LinearConstraintPtr constraint);
std::ostream& operator<<(std::ostream& stream, QuadraticConstraintPtr constraint);
std::ostream& operator<<(std::ostream& stream, NonlinearConstraintPtr constraint);

} // namespace SHOT