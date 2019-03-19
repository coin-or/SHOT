/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Shared.h"

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

    int index = -1;
    std::string name;

    ConstraintProperties properties;

    std::weak_ptr<Problem> ownerProblem;

    virtual bool isFulfilled(const VectorDouble& point) = 0;

    void takeOwnership(ProblemPtr owner);

    virtual std::ostream& print(std::ostream&) const = 0;

    virtual void updateProperties() = 0;
};

std::ostream& operator<<(std::ostream& stream, ConstraintPtr constraint);
std::ostream& operator<<(std::ostream& stream, const Constraint& constraint);

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

    virtual SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) = 0;
    virtual std::shared_ptr<Variables> getGradientSparsityPattern();

    // Returns the upper triagonal part of the Hessian matrix is sparse representation
    virtual SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) = 0;
    virtual std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> getHessianSparsityPattern();

    virtual NumericConstraintValue calculateNumericValue(const VectorDouble& point, double correction = 0.0);

    virtual bool isFulfilled(const VectorDouble& point) override;

    virtual std::shared_ptr<NumericConstraint> getPointer() = 0;

    virtual void updateProperties() = 0;

protected:
    virtual void initializeGradientSparsityPattern() = 0;
    virtual void initializeHessianSparsityPattern() = 0;
};

class LinearConstraint : public NumericConstraint
{
public:
    LinearTerms linearTerms;

    LinearConstraint(){};

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

    virtual double calculateFunctionValue(const VectorDouble& point) override;
    virtual Interval calculateFunctionValue(const IntervalVector& intervalVector);

    virtual bool isFulfilled(const VectorDouble& point) override;

    virtual SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes);

    // Returns the upper triagonal part of the Hessian matrix is sparse representation
    virtual SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes);

    virtual NumericConstraintValue calculateNumericValue(const VectorDouble& point, double correction = 0.0) override;

    virtual std::shared_ptr<NumericConstraint> getPointer() override;

    virtual void updateProperties() override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    virtual void initializeGradientSparsityPattern();
    virtual void initializeHessianSparsityPattern();
};

typedef std::shared_ptr<LinearConstraint> LinearConstraintPtr;
typedef std::vector<LinearConstraintPtr> LinearConstraints;

std::ostream& operator<<(std::ostream& stream, LinearConstraintPtr constraint);

class QuadraticConstraint : public LinearConstraint
{
public:
    QuadraticTerms quadraticTerms;

    QuadraticConstraint() : LinearConstraint(){};

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

    virtual double calculateFunctionValue(const VectorDouble& point) override;
    virtual Interval calculateFunctionValue(const IntervalVector& intervalVector);

    virtual bool isFulfilled(const VectorDouble& point) override;

    virtual SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes);

    // Returns the upper triagonal part of the Hessian matrix is sparse representation
    virtual SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes);

    virtual NumericConstraintValue calculateNumericValue(const VectorDouble& point, double correction = 0.0) override;

    virtual std::shared_ptr<NumericConstraint> getPointer() override;

    virtual void updateProperties() override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    virtual void initializeGradientSparsityPattern();
    virtual void initializeHessianSparsityPattern();
};

std::ostream& operator<<(std::ostream& stream, QuadraticConstraintPtr constraint);

class NonlinearConstraint : public QuadraticConstraint
{
public:
    NonlinearExpressionPtr nonlinearExpression;
    FactorableFunctionPtr factorableFunction;

    std::vector<std::pair<VariablePtr, FactorableFunction>> symbolicSparseJacobian;
    std::vector<std::pair<std::pair<VariablePtr, VariablePtr>, FactorableFunction>> symbolicSparseHessian;
    Variables variablesInNonlinearExpression;

    NonlinearConstraint(){};

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
    void add(NonlinearExpressionPtr expression);

    void updateFactorableFunction();

    virtual double calculateFunctionValue(const VectorDouble& point) override;

    virtual SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes);

    // Returns the upper triagonal part of the Hessian matrix is sparse representation
    virtual SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes);

    virtual Interval calculateFunctionValue(const IntervalVector& intervalVector);

    virtual bool isFulfilled(const VectorDouble& point) override;

    virtual NumericConstraintValue calculateNumericValue(const VectorDouble& point, double correction = 0.0) override;

    virtual std::shared_ptr<NumericConstraint> getPointer() override;

    virtual void updateProperties() override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    virtual void initializeGradientSparsityPattern();
    virtual void initializeHessianSparsityPattern();
};

std::ostream& operator<<(std::ostream& stream, NonlinearConstraintPtr constraint);
std::ostream& operator<<(std::ostream& stream, NumericConstraintPtr constraint);
std::ostream& operator<<(std::ostream& stream, LinearConstraintPtr constraint);
std::ostream& operator<<(std::ostream& stream, QuadraticConstraintPtr constraint);
std::ostream& operator<<(std::ostream& stream, NonlinearConstraintPtr constraint);

} // namespace SHOT