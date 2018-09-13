
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
#include <iostream>

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

    int constraintIndex = -1;
    std::string constraintName;

    ConstraintProperties properties;
    //virtual void updateProperties();

    virtual bool isFulfilled(const VectorDouble &point) = 0;

    virtual std::ostream &print(std::ostream &) const = 0;
    friend std::ostream &operator<<(std::ostream &stream, const Constraint &constraint)
    {
        stream << "[" << constraint.constraintIndex << "]";

        if (constraint.constraintName != "")
            stream << ' ' << constraint.constraintName;

        stream << ":\t";

        return constraint.print(stream); // polymorphic print via reference
    };
};

typedef std::shared_ptr<Constraint> ConstraintPtr;
typedef std::vector<ConstraintPtr> Constraints;

std::ostream &operator<<(std::ostream &stream, ConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

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

class NumericConstraint : public Constraint, public std::enable_shared_from_this<NumericConstraint>
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
        constrValue.functionValue = value;
        constrValue.isFulfilledRHS = (value <= valueRHS);
        constrValue.errorRHS = std::max(value - valueRHS, 0.0);

        constrValue.isFulfilledLHS = (value >= valueLHS);
        constrValue.errorLHS = std::max(valueLHS - value, 0.0);

        constrValue.isFulfilled = (constrValue.isFulfilledRHS && constrValue.isFulfilledLHS);
        constrValue.error = std::max(constrValue.errorRHS, constrValue.errorLHS);

        return constrValue;
    }

    virtual bool isFulfilled(const VectorDouble &point) override
    {
        auto constraintValue = calculateNumericValue(point);

        return (constraintValue.isFulfilledLHS && constraintValue.isFulfilledRHS);
    };

    virtual std::shared_ptr<NumericConstraint> getPointer() = 0;
};

std::ostream &operator<<(std::ostream &stream, NumericConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

class LinearConstraint : public NumericConstraint
{
  public:
    LinearTerms linearTerms;

    LinearConstraint(){};

    LinearConstraint(int index, std::string name, double LHS, double RHS)
    {
        constraintIndex = index;
        constraintName = name;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    LinearConstraint(int index, std::string name, LinearTerms linTerms, double LHS, double RHS)
    {
        constraintIndex = index;
        constraintName = name;
        linearTerms = linTerms;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    void add(LinearTerms terms)
    {
        if (linearTerms.terms.size() == 0)
        {
            linearTerms = terms;
        }
        else
        {
            for (auto T : terms.terms)
            {
                add(T);
            }
        }
    };

    void add(LinearTermPtr term)
    {
        linearTerms.terms.push_back(term);
    };

    virtual double calculateFunctionValue(const VectorDouble &point) override
    {
        double value = linearTerms.calculate(point);
        return value;
    };

    virtual bool isFulfilled(const VectorDouble &point) override
    {
        return NumericConstraint::isFulfilled(point);
    };

    virtual NumericConstraintValue calculateNumericValue(const VectorDouble &point) override
    {
        return NumericConstraint::calculateNumericValue(point);
    };

    virtual std::shared_ptr<NumericConstraint> getPointer() override
    {
        return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
    };

    std::ostream &print(std::ostream &stream) const override
    {
        if (valueLHS > -std::numeric_limits<double>::infinity())
            stream << valueLHS << " <= ";

        if (linearTerms.terms.size() > 0)
            stream << linearTerms;

        if (valueRHS < std::numeric_limits<double>::infinity())
            stream << " <= " << valueRHS;

        return stream;
    };
};

typedef std::shared_ptr<LinearConstraint> LinearConstraintPtr;

std::ostream &operator<<(std::ostream &stream, LinearConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

typedef std::vector<LinearConstraintPtr> LinearConstraints;

class QuadraticConstraint : public LinearConstraint
{
  public:
    QuadraticTerms quadraticTerms;

    QuadraticConstraint(){};

    QuadraticConstraint(int index, std::string name, double LHS, double RHS)
    {
        constraintIndex = index;
        constraintName = name;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    QuadraticConstraint(int index, std::string name, QuadraticTerms quadTerms, double LHS, double RHS)
    {
        constraintIndex = index;
        constraintName = name;
        quadraticTerms = quadTerms;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    QuadraticConstraint(int index, std::string name, LinearTerms linTerms, QuadraticTerms quadTerms, double LHS, double RHS)
    {
        constraintIndex = index;
        constraintName = name;
        linearTerms = linTerms;
        quadraticTerms = quadTerms;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    void add(LinearTerms terms)
    {
        LinearConstraint::add(terms);
    };

    void add(LinearTermPtr term)
    {
        LinearConstraint::add(term);
    };

    void add(QuadraticTerms terms)
    {
        if (quadraticTerms.terms.size() == 0)
        {
            quadraticTerms = terms;
        }
        else
        {
            for (auto T : terms.terms)
            {
                add(T);
            }
        }
    };

    void add(QuadraticTermPtr term)
    {
        quadraticTerms.terms.push_back(term);
    };

    virtual double calculateFunctionValue(const VectorDouble &point) override
    {
        double value = LinearConstraint::calculateFunctionValue(point);
        value += quadraticTerms.calculate(point);

        return value;
    };

    virtual bool isFulfilled(const VectorDouble &point) override
    {
        return NumericConstraint::isFulfilled(point);
    };

    virtual NumericConstraintValue calculateNumericValue(const VectorDouble &point) override
    {
        return NumericConstraint::calculateNumericValue(point);
    };

    virtual std::shared_ptr<NumericConstraint> getPointer() override
    {
        return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
    };

    std::ostream &print(std::ostream &stream) const override
    {
        if (valueLHS > -std::numeric_limits<double>::infinity())
            stream << valueLHS << " <= ";

        if (linearTerms.terms.size() > 0)
            stream << linearTerms;

        if (quadraticTerms.terms.size() > 0)
            stream << " +" << quadraticTerms;

        if (valueRHS < std::numeric_limits<double>::infinity())
            stream << " <= " << valueRHS;

        return stream;
    };
};

typedef std::shared_ptr<QuadraticConstraint> QuadraticConstraintPtr;

std::ostream &operator<<(std::ostream &stream, QuadraticConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

typedef std::vector<QuadraticConstraintPtr> QuadraticConstraints;

class NonlinearConstraint : public QuadraticConstraint
{
  public:
    NonlinearExpressionPtr nonlinearExpression;

    NonlinearConstraint(){};

    NonlinearConstraint(int index, std::string name, double LHS, double RHS)
    {
        constraintIndex = index;
        constraintName = name;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    NonlinearConstraint(int index, std::string name, NonlinearExpressionPtr expression, double LHS, double RHS)
    {
        constraintIndex = index;
        constraintName = name;
        nonlinearExpression = expression;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    NonlinearConstraint(int index, std::string name, QuadraticTerms quadTerms, NonlinearExpressionPtr expression, double LHS, double RHS)
    {
        constraintIndex = index;
        constraintName = name;
        quadraticTerms = quadTerms;
        nonlinearExpression = expression;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    NonlinearConstraint(int index, std::string name, LinearTerms linTerms, NonlinearExpressionPtr expression, double LHS, double RHS)
    {
        constraintIndex = index;
        constraintName = name;
        linearTerms = linTerms;
        nonlinearExpression = expression;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    NonlinearConstraint(int index, std::string name, LinearTerms linTerms, QuadraticTerms quadTerms, NonlinearExpressionPtr expression, double LHS, double RHS)
    {
        constraintIndex = index;
        constraintName = name;
        linearTerms = linTerms;
        quadraticTerms = quadTerms;
        nonlinearExpression = expression;
        valueLHS = LHS;
        valueRHS = RHS;
    };

    void add(LinearTerms terms)
    {
        LinearConstraint::add(terms);
    };

    void add(LinearTermPtr term)
    {
        LinearConstraint::add(term);
    };

    void add(QuadraticTerms terms)
    {
        QuadraticConstraint::add(terms);
    };

    void add(QuadraticTermPtr term)
    {
        QuadraticConstraint::add(term);
    };

    void add(NonlinearExpressionPtr expression)
    {
        if (nonlinearExpression != nullptr)
        {
            auto tmpExpr = nonlinearExpression;

            auto nonlinearExpression(new ExpressionPlus(tmpExpr, expression));
        }
        else
        {
            nonlinearExpression = expression;
        }
    };

    virtual double calculateFunctionValue(const VectorDouble &point) override
    {
        double value = QuadraticConstraint::calculateFunctionValue(point);
        value += nonlinearExpression->calculate(point);

        return value;
    };

    virtual bool isFulfilled(const VectorDouble &point) override
    {
        return NumericConstraint::isFulfilled(point);
    };

    virtual NumericConstraintValue calculateNumericValue(const VectorDouble &point) override
    {
        return NumericConstraint::calculateNumericValue(point);
    };

    virtual std::shared_ptr<NumericConstraint> getPointer() override
    {
        return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
    };

    std::ostream &print(std::ostream &stream) const override
    {
        if (valueLHS > -std::numeric_limits<double>::infinity())
            stream << valueLHS << " <= ";

        if (linearTerms.terms.size() > 0)
            stream << linearTerms;

        if (quadraticTerms.terms.size() > 0)
            stream << " +" << quadraticTerms;

        stream << " +" << nonlinearExpression;

        if (valueRHS < std::numeric_limits<double>::infinity())
            stream << " <= " << valueRHS;

        return stream;
    };
};

typedef std::shared_ptr<NonlinearConstraint> NonlinearConstraintPtr;

std::ostream &operator<<(std::ostream &stream, NonlinearConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

typedef std::vector<NonlinearConstraintPtr> NonlinearConstraints;

} // namespace SHOT