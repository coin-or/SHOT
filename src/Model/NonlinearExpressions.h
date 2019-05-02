/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @note The monotonicity and convexity identification is strongly influenced by that of Suspect
   (https://github.com/cog-imperial/suspect) by Francesco Ceccon, Imperial College London

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Enums.h"
#include "Variables.h"

#include "../Utilities.h"

#include "ffunc.hpp"

namespace SHOT
{

typedef mc::FFVar FactorableFunction;
typedef std::shared_ptr<FactorableFunction> FactorableFunctionPtr;
typedef mc::Interval Interval;
typedef std::vector<Interval> IntervalVector;

enum class E_NonlinearExpressionTypes
{
    Constant,
    Variable,
    Negate,
    Invert,
    SquareRoot,
    Log,
    Exp,
    Square,
    Cos,
    Sin,
    Tan,
    ArcCos,
    ArcSin,
    ArcTan,
    Abs,
    Divide,
    Power,
    Sum,
    Product
};

inline E_Monotonicity negateMonotonicity(E_Monotonicity monotonicity)
{
    E_Monotonicity resultMonotonicity;

    switch(monotonicity)
    {
    case(E_Monotonicity::Constant):
        resultMonotonicity = E_Monotonicity::Constant;
        break;

    case(E_Monotonicity::Nondecreasing):
        resultMonotonicity = E_Monotonicity::Nonincreasing;
        break;

    case(E_Monotonicity::Nonincreasing):
        resultMonotonicity = E_Monotonicity::Nondecreasing;
        break;

    case(E_Monotonicity::Unknown):
        resultMonotonicity = E_Monotonicity::Unknown;
        break;

    default:
        resultMonotonicity = E_Monotonicity::NotSet;
        break;
    }

    return resultMonotonicity;
};

inline E_Convexity getConvexityTimesWithConstantFunction(E_Convexity nonconstantConvexity, Interval constantBound)
{
    if(nonconstantConvexity == E_Convexity::Linear)
        return E_Convexity::Linear;

    if(nonconstantConvexity == E_Convexity::Convex && constantBound.l() >= 0.0)
        return E_Convexity::Convex;

    if(nonconstantConvexity == E_Convexity::Concave && constantBound.u() <= 0.0)
        return E_Convexity::Convex;

    if(nonconstantConvexity == E_Convexity::Concave && constantBound.l() >= 0.0)
        return E_Convexity::Concave;

    if(nonconstantConvexity == E_Convexity::Convex && constantBound.u() <= 0.0)
        return E_Convexity::Concave;

    return E_Convexity::Unknown;
}

inline E_Monotonicity getMonotonicityTimesWithConstantFunction(
    E_Monotonicity nonconstantMonotonicity, Interval nonconstantBound, Interval constantBound)
{
    if(nonconstantMonotonicity == E_Monotonicity::Nondecreasing && constantBound.l() >= 0.0)
        return E_Monotonicity::Nondecreasing;

    if(nonconstantMonotonicity == E_Monotonicity::Nonincreasing && constantBound.u() <= 0.0)
        return E_Monotonicity::Nondecreasing;

    if(nonconstantMonotonicity == E_Monotonicity::Nondecreasing && constantBound.u() <= 0.0)
        return E_Monotonicity::Nonincreasing;

    if(nonconstantMonotonicity == E_Monotonicity::Nonincreasing && constantBound.l() >= 0.0)
        return E_Monotonicity::Nonincreasing;

    return E_Monotonicity::Unknown;
};

class NonlinearExpression
{
public:
    std::weak_ptr<Problem> ownerProblem;

    virtual inline void takeOwnership(ProblemPtr owner) { ownerProblem = owner; }

    virtual double calculate(const VectorDouble& point) const = 0;
    virtual Interval calculate(const IntervalVector& intervalVector) const = 0;
    virtual Interval getBounds() const = 0;

    virtual FactorableFunction getFactorableFunction() = 0;

    virtual std::ostream& print(std::ostream&) const = 0;

    virtual E_NonlinearExpressionTypes getType() const = 0;

    virtual E_Convexity getConvexity() const = 0;
    virtual E_Monotonicity getMonotonicity() const = 0;

    virtual int getNumberOfChildren() const = 0;

    virtual void appendNonlinearVariables(Variables& nonlinearVariables) = 0;

    inline friend std::ostream& operator<<(std::ostream& stream, const NonlinearExpression& expr)
    {
        return expr.print(stream); // polymorphic print via reference
    };

    virtual bool operator==(const NonlinearExpression& rhs) const = 0;
};

typedef std::shared_ptr<NonlinearExpression> NonlinearExpressionPtr;

inline std::ostream& operator<<(std::ostream& stream, NonlinearExpressionPtr expr)
{
    if(expr != nullptr)
    {
        stream << *expr;
    }

    return stream;
};

class NonlinearExpressions : private std::vector<NonlinearExpressionPtr>
{
public:
    using std::vector<NonlinearExpressionPtr>::operator[];

    using std::vector<NonlinearExpressionPtr>::at;
    using std::vector<NonlinearExpressionPtr>::begin;
    using std::vector<NonlinearExpressionPtr>::clear;
    using std::vector<NonlinearExpressionPtr>::end;
    using std::vector<NonlinearExpressionPtr>::erase;
    using std::vector<NonlinearExpressionPtr>::push_back;
    using std::vector<NonlinearExpressionPtr>::reserve;
    using std::vector<NonlinearExpressionPtr>::resize;
    using std::vector<NonlinearExpressionPtr>::size;

    inline void add(NonlinearExpressionPtr expression) { (*this).push_back(expression); };

    // NonlinearExpressionPtr get(int i) { return expressions.at(i); };

    // inline size_t size() const { return expressions.size(); };
};

class ExpressionConstant : public NonlinearExpression
{
public:
    double constant = 0;
    ExpressionConstant(double constant) : constant(constant){};

    inline double calculate(const VectorDouble& point) const override { return constant; };

    inline Interval calculate(const IntervalVector& intervalVector) const override { return (Interval(constant)); };

    inline Interval getBounds() const override { return Interval(constant); };

    inline FactorableFunction getFactorableFunction() override { return constant; };

    inline std::ostream& print(std::ostream& stream) const override { return stream << constant; };

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Constant; };

    inline E_Convexity getConvexity() const override { return E_Convexity::Linear; };
    inline E_Monotonicity getMonotonicity() const override { return E_Monotonicity::Constant; };

    inline int getNumberOfChildren() const override { return 0; }

    inline void appendNonlinearVariables(Variables& nonlinearVariables) override{};

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (static_cast<const ExpressionConstant&>(rhs).constant == constant);
    };
};

class ExpressionVariable : public NonlinearExpression
{
public:
    VariablePtr variable;

    ExpressionVariable() { variable->isNonlinear = true; };

    ExpressionVariable(VariablePtr variable) : variable(variable) { variable->isNonlinear = true; };

    inline double calculate(const VectorDouble& point) const override { return (variable->calculate(point)); };

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (variable->calculate(intervalVector));
    };

    inline FactorableFunction getFactorableFunction() override
    {
        return *(variable->factorableFunctionVariable.get());
    };

    inline Interval getBounds() const override { return (variable->getBound()); };

    inline std::ostream& print(std::ostream& stream) const override { return stream << variable->name; };

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Variable; };

    inline E_Convexity getConvexity() const override { return E_Convexity::Linear; };
    inline E_Monotonicity getMonotonicity() const override { return E_Monotonicity::Nondecreasing; };

    inline int getNumberOfChildren() const override { return 0; }

    inline void appendNonlinearVariables(Variables& nonlinearVariables) override
    {
        if(std::find(nonlinearVariables.begin(), nonlinearVariables.end(), variable) == nonlinearVariables.end())
            nonlinearVariables.push_back(variable);
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (static_cast<const ExpressionVariable&>(rhs).variable == variable);
    };
};

typedef std::shared_ptr<ExpressionVariable> ExpressionVariablePtr;

class ExpressionUnary : public NonlinearExpression
{
public:
    NonlinearExpressionPtr child;

    inline void takeOwnership(ProblemPtr owner) override
    {
        ownerProblem = owner;
        child->takeOwnership(owner);
    }

    double calculate(const VectorDouble& point) const override = 0;
    Interval calculate(const IntervalVector& intervalVector) const override = 0;
    FactorableFunction getFactorableFunction() override = 0;
    E_NonlinearExpressionTypes getType() const override = 0;

    inline int getNumberOfChildren() const override { return 1; }

    inline void appendNonlinearVariables(Variables& nonlinearVariables) override
    {
        child->appendNonlinearVariables(nonlinearVariables);
    };
};

class ExpressionBinary : public NonlinearExpression
{
private:
    bool isAssociative = false;

public:
    NonlinearExpressionPtr firstChild;
    NonlinearExpressionPtr secondChild;

    inline void takeOwnership(ProblemPtr owner) override
    {
        ownerProblem = owner;
        firstChild->takeOwnership(owner);
        secondChild->takeOwnership(owner);
    }

    double calculate(const VectorDouble& point) const override = 0;
    Interval calculate(const IntervalVector& intervalVector) const override = 0;
    FactorableFunction getFactorableFunction() override = 0;
    E_NonlinearExpressionTypes getType() const override = 0;

    inline int getNumberOfChildren() const override { return 2; }

    inline void appendNonlinearVariables(Variables& nonlinearVariables) override
    {
        firstChild->appendNonlinearVariables(nonlinearVariables);
        secondChild->appendNonlinearVariables(nonlinearVariables);
    };
};

class ExpressionGeneral : public NonlinearExpression
{
public:
    NonlinearExpressions children;

    inline void takeOwnership(ProblemPtr owner) override
    {
        ownerProblem = owner;

        for(auto& C : children)
            C->takeOwnership(owner);
    }

    double calculate(const VectorDouble& point) const override = 0;
    Interval calculate(const IntervalVector& intervalVector) const override = 0;
    FactorableFunction getFactorableFunction() override = 0;
    E_NonlinearExpressionTypes getType() const override = 0;

    inline int getNumberOfChildren() const override { return children.size(); }

    inline void appendNonlinearVariables(Variables& nonlinearVariables) override
    {
        for(auto& C : children)
            C->appendNonlinearVariables(nonlinearVariables);
    };
};

// Begin unary operations

class ExpressionNegate : public ExpressionUnary
{
public:
    ExpressionNegate() = default;

    ExpressionNegate(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (-child->calculate(point)); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (-child->calculate(intervalVector));
    }

    inline Interval getBounds() const override { return (-child->getBounds()); };

    inline FactorableFunction getFactorableFunction() override { return (-child->getFactorableFunction()); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "(-" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Negate; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();
        E_Convexity resultConvexity;

        switch(childConvexity)
        {
        case(E_Convexity::Linear):
            resultConvexity = E_Convexity::Linear;
            break;

        case(E_Convexity::Convex):
            resultConvexity = E_Convexity::Concave;
            break;

        case(E_Convexity::Concave):
            resultConvexity = E_Convexity::Convex;
            break;

        case(E_Convexity::Nonconvex):
            resultConvexity = E_Convexity::Nonconvex;
            break;

        case(E_Convexity::Unknown):
            resultConvexity = E_Convexity::Unknown;
            break;

        default:
            resultConvexity = E_Convexity::NotSet;
            break;
        }

        return resultConvexity;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        E_Monotonicity resultMonotonicity = negateMonotonicity(childMonotonicity);

        return resultMonotonicity;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionNegate&>(rhs).child.get() == child.get());
    };
};

class ExpressionInvert : public ExpressionUnary
{
public:
    ExpressionInvert() = default;

    ExpressionInvert(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (1.0 / child->calculate(point)); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (1.0 / child->calculate(intervalVector));
    }

    inline Interval getBounds() const override { return (1.0 / child->getBounds()); }

    inline FactorableFunction getFactorableFunction() override { return (1 / child->getFactorableFunction()); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "1/(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Invert; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();
        auto bounds = child->getBounds();

        if((bounds.l() <= 0.0 && bounds.u() >= 0))
            return E_Convexity::Unknown;

        if(bounds.l() > 0 && childConvexity == E_Convexity::Concave)
            return E_Convexity::Convex;

        if(bounds.l() > 0 && childConvexity == E_Convexity::Linear)
            return E_Convexity::Convex;

        if(bounds.l() < 0 && childConvexity == E_Convexity::Convex)
            return E_Convexity::Concave;

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        auto bounds = child->getBounds();

        if(childMonotonicity == E_Monotonicity::Constant && (bounds.l() == 0.0 || bounds.u()))
            return E_Monotonicity::Unknown;

        if(childMonotonicity == E_Monotonicity::Constant)
            return E_Monotonicity::Constant;

        if(childMonotonicity == E_Monotonicity::Nonincreasing)
            return E_Monotonicity::Nondecreasing;

        if(childMonotonicity == E_Monotonicity::Nondecreasing)
            return E_Monotonicity::Nonincreasing;

        return E_Monotonicity::Unknown;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionInvert&>(rhs).child.get() == child.get());
    };
};

class ExpressionSquareRoot : public ExpressionUnary
{
public:
    ExpressionSquareRoot() = default;

    ExpressionSquareRoot(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (sqrt(child->calculate(point))); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (sqrt(child->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (sqrt(child->getBounds())); }

    inline FactorableFunction getFactorableFunction() override { return (sqrt(child->getFactorableFunction())); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "sqrt(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::SquareRoot; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();
        auto childBounds = child->getBounds();

        if(childBounds.l() >= 0 && childConvexity == E_Convexity::Concave)
            return E_Convexity::Concave;

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        return childMonotonicity;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionSquareRoot&>(rhs).child.get() == child.get());
    };
};

class ExpressionLog : public ExpressionUnary
{
public:
    ExpressionLog() = default;

    ExpressionLog(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (log(child->calculate(point))); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (log(child->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (log(child->getBounds())); }

    inline FactorableFunction getFactorableFunction() override { return (log(child->getFactorableFunction())); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "log(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Log; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();
        auto childBounds = child->getBounds();

        if(childBounds.l() >= 0 && (childConvexity == E_Convexity::Concave || childConvexity == E_Convexity::Linear))
            return E_Convexity::Concave;

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        return childMonotonicity;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionLog&>(rhs).child.get() == child.get());
    };
};

class ExpressionExp : public ExpressionUnary
{
public:
    ExpressionExp() = default;

    ExpressionExp(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (exp(child->calculate(point))); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (exp(child->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (exp(child->getBounds())); }

    inline FactorableFunction getFactorableFunction() override { return (exp(child->getFactorableFunction())); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "exp(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Exp; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();

        if(childConvexity == E_Convexity::Convex || childConvexity == E_Convexity::Linear)
            return E_Convexity::Convex;

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        return childMonotonicity;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionExp&>(rhs).child.get() == child.get());
    };
};

class ExpressionSquare : public ExpressionUnary
{
public:
    ExpressionSquare() = default;

    ExpressionSquare(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override
    {
        auto value = child->calculate(point);
        return (value * value);
    }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        auto value = child->calculate(intervalVector);
        return (value * value);
    }

    inline Interval getBounds() const override
    {
        auto value = child->getBounds();
        return (value * value);
    }

    inline FactorableFunction getFactorableFunction() override
    {
        return (child->getFactorableFunction() * child->getFactorableFunction());
    }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "(" << child << ")^2";
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Square; }

    inline E_Convexity getConvexity() const override
    {
        auto baseBounds = child->getBounds();
        auto baseConvexity = child->getConvexity();

        if(baseConvexity == E_Convexity::Linear)
            return E_Convexity::Convex;

        if(baseConvexity == E_Convexity::Convex && baseBounds.l() >= 0)
            return E_Convexity::Convex;

        if(baseConvexity == E_Convexity::Concave && baseBounds.u() <= 0)
            return E_Convexity::Convex;

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();

        auto bounds = child->getBounds();

        if(childMonotonicity == E_Monotonicity::Constant)
            return E_Monotonicity::Constant;

        if((childMonotonicity == E_Monotonicity::Nondecreasing && bounds.l() >= 0.0)
            || (childMonotonicity == E_Monotonicity::Nonincreasing && bounds.u() <= 0.0))
            return E_Monotonicity::Nondecreasing;

        if((childMonotonicity == E_Monotonicity::Nonincreasing && bounds.l() >= 0.0)
            || (childMonotonicity == E_Monotonicity::Nondecreasing && bounds.u() <= 0.0))
            return E_Monotonicity::Nonincreasing;

        return E_Monotonicity::Unknown;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionSquare&>(rhs).child.get() == child.get());
    };
};

class ExpressionSin : public ExpressionUnary
{
public:
    ExpressionSin() = default;

    ExpressionSin(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (sin(child->calculate(point))); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (sin(child->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (sin(child->getBounds())); }

    inline FactorableFunction getFactorableFunction() override { return (sin(child->getFactorableFunction())); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "sin(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Sin; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();
        auto childBounds = child->getBounds();

        if(childBounds.u() > M_PI)
            return E_Convexity::Unknown;

        auto sinBounds = sin(childBounds);

        if(sinBounds.l() * sinBounds.u() < 0)
            return E_Convexity::Unknown;

        auto cosBounds = cos(childBounds);

        if(sinBounds.l() >= 0)
        {
            if(childConvexity == E_Convexity::Linear)
                return E_Convexity::Concave;

            if(childConvexity == E_Convexity::Convex && cosBounds.u() <= 0)
                return E_Convexity::Concave;

            if(childConvexity == E_Convexity::Concave && cosBounds.u() >= 0)
                return E_Convexity::Concave;
        }
        else if(sinBounds.u() <= 0)
        {
            if(childConvexity == E_Convexity::Linear)
                return E_Convexity::Convex;

            if(childConvexity == E_Convexity::Concave && cosBounds.u() <= 0)
                return E_Convexity::Convex;

            if(childConvexity == E_Convexity::Convex && cosBounds.u() >= 0)
                return E_Convexity::Convex;
        }

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        auto childBounds = child->getBounds();

        auto cosBounds = cos(childBounds);

        if(childMonotonicity == E_Monotonicity::Nondecreasing && cosBounds.l() >= 0)
            return E_Monotonicity::Nondecreasing;

        if(childMonotonicity == E_Monotonicity::Nonincreasing && cosBounds.u() <= 0)
            return E_Monotonicity::Nondecreasing;

        if(childMonotonicity == E_Monotonicity::Nonincreasing && cosBounds.l() >= 0)
            return E_Monotonicity::Nonincreasing;

        if(childMonotonicity == E_Monotonicity::Nondecreasing && cosBounds.u() <= 0)
            return E_Monotonicity::Nonincreasing;

        return E_Monotonicity::Unknown;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionSin&>(rhs).child.get() == child.get());
    };
};

class ExpressionCos : public ExpressionUnary
{
public:
    ExpressionCos() = default;

    ExpressionCos(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (cos(child->calculate(point))); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (cos(child->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (cos(child->getBounds())); }

    inline FactorableFunction getFactorableFunction() override { return (cos(child->getFactorableFunction())); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "cos(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Cos; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();
        auto childBounds = child->getBounds();

        if(childBounds.u() > M_PI)
            return E_Convexity::Unknown;

        auto cosBounds = cos(childBounds);

        if(cosBounds.l() * cosBounds.u() < 0)
            return E_Convexity::Unknown;

        auto sinBounds = sin(childBounds);

        if(cosBounds.l() >= 0)
        {
            if(childConvexity == E_Convexity::Linear)
                return E_Convexity::Concave;

            if(childConvexity == E_Convexity::Convex && sinBounds.u() <= 0)
                return E_Convexity::Concave;

            if(childConvexity == E_Convexity::Concave && sinBounds.u() >= 0)
                return E_Convexity::Concave;
        }
        else if(cosBounds.u() <= 0)
        {
            if(childConvexity == E_Convexity::Linear)
                return E_Convexity::Convex;

            if(childConvexity == E_Convexity::Concave && sinBounds.u() <= 0)
                return E_Convexity::Convex;

            if(childConvexity == E_Convexity::Convex && sinBounds.u() >= 0)
                return E_Convexity::Convex;
        }

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        auto childBounds = child->getBounds();

        auto sinBounds = sin(childBounds);

        if(childMonotonicity == E_Monotonicity::Nonincreasing && sinBounds.l() >= 0)
            return E_Monotonicity::Nondecreasing;

        if(childMonotonicity == E_Monotonicity::Nondecreasing && sinBounds.u() <= 0)
            return E_Monotonicity::Nondecreasing;

        if(childMonotonicity == E_Monotonicity::Nondecreasing && sinBounds.l() >= 0)
            return E_Monotonicity::Nonincreasing;

        if(childMonotonicity == E_Monotonicity::Nonincreasing && sinBounds.u() <= 0)
            return E_Monotonicity::Nonincreasing;

        return E_Monotonicity::Unknown;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionCos&>(rhs).child.get() == child.get());
    };
};

class ExpressionTan : public ExpressionUnary
{
public:
    ExpressionTan() = default;

    ExpressionTan(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (tan(child->calculate(point))); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (tan(child->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (tan(child->getBounds())); }

    inline FactorableFunction getFactorableFunction() override { return (tan(child->getFactorableFunction())); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "tan(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Tan; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();
        auto childBounds = child->getBounds();

        if(2.0 * (childBounds.u() - childBounds.l()) > M_PI)
            return E_Convexity::Unknown;

        auto tanBounds = tan(childBounds);

        if(tanBounds.l() * tanBounds.u() < 0)
            return E_Convexity::Unknown;

        if(tanBounds.l() >= 0 && childConvexity == E_Convexity::Convex)

            return E_Convexity::Convex;

        if(tanBounds.u() <= 0 && childConvexity == E_Convexity::Concave)

            return E_Convexity::Concave;

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        return childMonotonicity;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionTan&>(rhs).child.get() == child.get());
    };
};

class ExpressionArcSin : public ExpressionUnary
{
public:
    ExpressionArcSin() = default;

    ExpressionArcSin(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (asin(child->calculate(point))); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (asin(child->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (asin(child->getBounds())); }

    inline FactorableFunction getFactorableFunction() override { return (asin(child->getFactorableFunction())); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "arcsin(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::ArcSin; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();
        auto childBounds = child->getBounds();

        if(childConvexity == E_Convexity::Concave && childBounds.l() >= -1.0 && childBounds.u() <= 0.0)
            return E_Convexity::Concave;

        if(childConvexity == E_Convexity::Convex && childBounds.l() >= 0.0 && childBounds.u() <= 1.0)
            return E_Convexity::Convex;

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        return childMonotonicity;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionArcSin&>(rhs).child.get() == child.get());
    };
};

class ExpressionArcCos : public ExpressionUnary
{
public:
    ExpressionArcCos() = default;

    ExpressionArcCos(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (acos(child->calculate(point))); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (acos(child->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (acos(child->getBounds())); }

    inline FactorableFunction getFactorableFunction() override { return (acos(child->getFactorableFunction())); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "arccos(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::ArcCos; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();
        auto childBounds = child->getBounds();

        if(childConvexity == E_Convexity::Convex && childBounds.l() >= -1.0 && childBounds.u() <= 0.0)
            return E_Convexity::Convex;

        if(childConvexity == E_Convexity::Concave && childBounds.l() >= 0.0 && childBounds.u() <= 1.0)
            return E_Convexity::Concave;

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        auto resultMonotonicity = negateMonotonicity(childMonotonicity);
        return resultMonotonicity;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionArcCos&>(rhs).child.get() == child.get());
    };
};

class ExpressionArcTan : public ExpressionUnary
{
public:
    ExpressionArcTan() = default;

    ExpressionArcTan(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (atan(child->calculate(point))); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (atan(child->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (atan(child->getBounds())); }

    inline FactorableFunction getFactorableFunction() override { return (atan(child->getFactorableFunction())); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "arctan(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::ArcTan; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();
        auto childBounds = child->getBounds();

        if(childConvexity == E_Convexity::Convex && childBounds.u() <= 0.0)
            return E_Convexity::Convex;

        if(childConvexity == E_Convexity::Concave && childBounds.l() >= 0.0)
            return E_Convexity::Concave;

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        return childMonotonicity;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionArcTan&>(rhs).child.get() == child.get());
    };
};

class ExpressionAbs : public ExpressionUnary
{
public:
    ExpressionAbs() = default;

    ExpressionAbs(NonlinearExpressionPtr childExpression) { child = childExpression; }

    inline double calculate(const VectorDouble& point) const override { return (abs(child->calculate(point))); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (abs(child->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (abs(child->getBounds())); }

    inline FactorableFunction getFactorableFunction() override { return (fabs(child->getFactorableFunction())); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "abs(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Abs; }

    inline E_Convexity getConvexity() const override
    {
        auto childConvexity = child->getConvexity();
        auto childBounds = child->getBounds();

        if(childConvexity == E_Convexity::Linear)
            return E_Convexity::Convex;

        if(childConvexity == E_Convexity::Convex)
        {
            if(childBounds.l() >= 0)
                return E_Convexity::Convex;

            if(childBounds.u() <= 0)
                return E_Convexity::Concave;
        }
        else if(childConvexity == E_Convexity::Concave)
        {
            if(childBounds.u() <= 0)
                return E_Convexity::Convex;

            if(childBounds.l() >= 0)
                return E_Convexity::Concave;
        }

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto childMonotonicity = child->getMonotonicity();
        auto childBounds = child->getBounds();

        if(childMonotonicity == E_Monotonicity::Constant)
            return E_Monotonicity::Constant;

        if(childBounds.l() >= 0.0)
            return childMonotonicity;

        if(childBounds.u() < 0.0)
            return negateMonotonicity(childMonotonicity);

        return E_Monotonicity::Unknown;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        return (dynamic_cast<const ExpressionAbs&>(rhs).child.get() == child.get());
    };
};

// End unary operations

// Begin binary operations

class ExpressionDivide : public ExpressionBinary
{
private:
    bool isAssociative = false;

public:
    ExpressionDivide() = default;

    ExpressionDivide(NonlinearExpressionPtr childExpression1, NonlinearExpressionPtr childExpression2)
    {
        firstChild = childExpression1;
        secondChild = childExpression2;
    }

    inline double calculate(const VectorDouble& point) const override
    {
        return (firstChild->calculate(point) / secondChild->calculate(point));
    }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (firstChild->calculate(intervalVector) / secondChild->calculate(intervalVector));
    }

    inline Interval getBounds() const override { return (firstChild->getBounds() / secondChild->getBounds()); }

    inline FactorableFunction getFactorableFunction() override
    {
        return (firstChild->getFactorableFunction() / secondChild->getFactorableFunction());
    }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << firstChild << '/' << secondChild;
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Divide; }

    inline E_Convexity getConvexity() const override
    {
        auto child2Convexity = secondChild->getConvexity();

        auto child1Monotonicity = firstChild->getMonotonicity();
        auto child2Monotonicity = secondChild->getMonotonicity();

        auto bounds1 = firstChild->getBounds();
        auto bounds2 = secondChild->getBounds();

        if(child2Monotonicity == E_Monotonicity::Constant)
        {
            auto child1Convexity = firstChild->getConvexity();

            if(child1Convexity == E_Convexity::Convex && bounds2.l() > 0)
                return E_Convexity::Convex;

            if(child1Convexity == E_Convexity::Concave && bounds2.u() < 0)
                return E_Convexity::Convex;

            if(child1Convexity == E_Convexity::Concave && bounds2.l() > 0)
                return E_Convexity::Concave;

            if(child1Convexity == E_Convexity::Convex && bounds2.u() < 0)
                return E_Convexity::Concave;
        }

        if(child1Monotonicity == E_Monotonicity::Constant)
        {
            auto child2Convexity = secondChild->getConvexity();

            if(bounds1.l() >= 0)
            {
                if(bounds2.l() > 0 && child2Convexity == E_Convexity::Concave)
                    return E_Convexity::Convex;

                if(bounds2.l() > 0 && child2Convexity == E_Convexity::Linear)
                    return E_Convexity::Convex;

                if(bounds2.u() < 0 && child2Convexity == E_Convexity::Convex)
                    return E_Convexity::Concave;

                if(bounds2.u() < 0 && child2Convexity == E_Convexity::Linear)
                    return E_Convexity::Concave;
            }
            else
            {
                if(bounds2.u() < 0 && child2Convexity == E_Convexity::Convex)
                    return E_Convexity::Convex;

                if(bounds2.u() < 0 && child2Convexity == E_Convexity::Linear)
                    return E_Convexity::Convex;

                if(bounds2.l() > 0 && child2Convexity == E_Convexity::Concave)
                    return E_Convexity::Concave;

                if(bounds2.l() > 0 && child2Convexity == E_Convexity::Linear)
                    return E_Convexity::Concave;
            }
        }

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto child1Monotonicity = firstChild->getMonotonicity();
        auto child2Monotonicity = secondChild->getMonotonicity();

        auto bounds1 = firstChild->getBounds();
        auto bounds2 = secondChild->getBounds();

        if(child2Monotonicity == E_Monotonicity::Constant && (bounds2.l() == 0.0 || bounds2.u()))
            return E_Monotonicity::Unknown;

        if(child1Monotonicity == E_Monotonicity::Constant && child2Monotonicity == E_Monotonicity::Constant)
            return E_Monotonicity::Constant;

        if(child2Monotonicity == E_Monotonicity::Constant)
        {
            if(child1Monotonicity == E_Monotonicity::Nondecreasing && bounds2.l() >= 0)
                return E_Monotonicity::Nondecreasing;

            if(child1Monotonicity == E_Monotonicity::Nonincreasing && bounds2.u() <= 0)
                return E_Monotonicity::Nondecreasing;

            if(child1Monotonicity == E_Monotonicity::Nondecreasing && bounds2.u() <= 0)
                return E_Monotonicity::Nonincreasing;

            if(child1Monotonicity == E_Monotonicity::Nonincreasing && bounds2.l() >= 0)
                return E_Monotonicity::Nonincreasing;
        }

        bool nonDecreasingCond1 = (child1Monotonicity == E_Monotonicity::Nondecreasing && bounds2.l() >= 0)
            || (child1Monotonicity == E_Monotonicity::Nonincreasing && bounds2.u() <= 0);

        bool nonDecreasingCond2 = (child2Monotonicity == E_Monotonicity::Nonincreasing && bounds1.l() >= 0)
            || (child2Monotonicity == E_Monotonicity::Nondecreasing && bounds1.u() <= 0);

        bool nonIncreasingCond1 = (child1Monotonicity == E_Monotonicity::Nonincreasing && bounds2.l() >= 0)
            || (child1Monotonicity == E_Monotonicity::Nondecreasing && bounds2.u() <= 0);

        bool nonIncreasingCond2 = (child2Monotonicity == E_Monotonicity::Nondecreasing && bounds1.l() >= 0)
            || (child2Monotonicity == E_Monotonicity::Nonincreasing && bounds1.u() <= 0);

        if(nonDecreasingCond1 && nonDecreasingCond2)
            return E_Monotonicity::Nondecreasing;

        if(nonIncreasingCond1 && nonIncreasingCond2)
            return E_Monotonicity::Nonincreasing;

        return E_Monotonicity::Unknown;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        auto expression = dynamic_cast<const ExpressionDivide&>(rhs);

        return (expression.firstChild.get() == firstChild.get() && expression.secondChild.get() == secondChild.get());
    };
};

class ExpressionPower : public ExpressionBinary
{
private:
    bool isAssociative = false;

public:
    ExpressionPower() = default;

    ExpressionPower(NonlinearExpressionPtr childExpression1, NonlinearExpressionPtr childExpression2)
    {
        firstChild = childExpression1;
        secondChild = childExpression2;
    }

    inline double calculate(const VectorDouble& point) const override
    {
        auto firstChildValue = firstChild->calculate(point);
        auto secondChildValue = secondChild->calculate(point);

        if(std::abs(firstChildValue - 0.0) <= 1e-10 * std::abs(firstChildValue))
        {
            return 0.0;
        }

        if(std::abs(firstChildValue - 1.0) <= 1e-10 * std::abs(firstChildValue))
        {
            return 1.0;
        }

        if(std::abs(secondChildValue - 0.0) <= 1e-10 * std::abs(firstChildValue))
        {
            return 1.0;
        }

        if(std::abs(secondChildValue - 1.0) <= 1e-10 * std::abs(firstChildValue))
        {
            return firstChildValue;
        }

        return (pow(firstChildValue, secondChildValue));
    }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (pow(firstChild->calculate(intervalVector), secondChild->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (pow(firstChild->getBounds(), secondChild->getBounds())); }

    inline FactorableFunction getFactorableFunction() override
    {
        // Special logic for integer powers
        if(secondChild->getType() == E_NonlinearExpressionTypes::Constant)
        {
            auto constantValue = std::dynamic_pointer_cast<ExpressionConstant>(secondChild)->constant;

            double intpart;

            if(std::modf(constantValue, &intpart) == 0.0)
            {
                int power = (int)constantValue;

                FactorableFunction result = firstChild->getFactorableFunction();

                for(int i = 1; i < power; i++)
                {
                    result *= firstChild->getFactorableFunction();
                }

                return (result);
            }
        }

        return (pow(firstChild->getFactorableFunction(), secondChild->getFactorableFunction()));
    }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << '(' << firstChild << ")^(" << secondChild << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Power; }

    inline E_Convexity getConvexity() const override
    {
        auto baseMonotonicity = firstChild->getMonotonicity();
        auto exponentMonotonicity = secondChild->getMonotonicity();

        if(exponentMonotonicity == E_Monotonicity::Constant)
        {
            auto baseBounds = firstChild->getBounds();
            auto baseConvexity = firstChild->getConvexity();

            double exponentValue = secondChild->getBounds().l();

            return getConvexityConstantExponent(exponentValue, baseConvexity, baseBounds);
        }
        else if(baseMonotonicity == E_Monotonicity::Constant)
        {
            auto exponentBounds = secondChild->getBounds();
            auto exponentConvexity = secondChild->getConvexity();

            double baseValue = firstChild->getBounds().l();

            return getConvexityConstantBase(baseValue, exponentConvexity);
        }

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        auto baseMonotonicity = firstChild->getMonotonicity();
        auto exponentMonotonicity = secondChild->getMonotonicity();

        if(exponentMonotonicity == E_Monotonicity::Constant)
        {
            double exponentValue = secondChild->getBounds().l();
            auto baseBounds = firstChild->getBounds();

            return getMonotonicityConstantExponent(exponentValue, baseMonotonicity, baseBounds);
        }

        if(baseMonotonicity == E_Monotonicity::Constant)
        {
            double baseValue = firstChild->getBounds().l();
            auto exponentBounds = secondChild->getBounds();

            return getMonotonicityConstantBase(baseValue, exponentMonotonicity, exponentBounds);
        }

        return E_Monotonicity::Unknown;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return (false);

        auto expression = dynamic_cast<const ExpressionPower&>(rhs);

        return (expression.firstChild.get() == firstChild.get() && expression.secondChild.get() == secondChild.get());
    };

private:
    inline E_Convexity getConvexityConstantBase(double baseValue, E_Convexity exponentConvexity) const
    {
        if(baseValue > 0.0 && baseValue < 1.0)
        {
            if(exponentConvexity == E_Convexity::Concave)
                return E_Convexity::Convex;
        }
        else if(baseValue >= 1.0)
            if(exponentConvexity == E_Convexity::Convex)
                return E_Convexity::Convex;

        return E_Convexity::Unknown;
    };

    inline E_Convexity getConvexityConstantExponent(
        double exponentValue, E_Convexity baseConvexity, Interval baseBounds) const
    {
        if(std::abs(exponentValue - 0.0) <= 1e-10 * std::abs(exponentValue))
            return E_Convexity::Linear;

        if(std::abs(exponentValue - 1.0) <= 1e-10 * std::abs(exponentValue))
            return baseConvexity;

        double intpart;
        bool isInteger = (std::modf(exponentValue, &intpart) == 0.0);
        bool integerValue = round(intpart);
        bool isEven = (integerValue % 2 == 0);

        if(isInteger && isEven)
        {
            if(exponentValue > 0)
            {
                if(baseConvexity == E_Convexity::Linear)
                    return E_Convexity::Convex;

                if(baseConvexity == E_Convexity::Convex && baseBounds.l() >= 0)
                    return E_Convexity::Convex;

                if(baseConvexity == E_Convexity::Concave && baseBounds.u() <= 0)
                    return E_Convexity::Convex;
            }
            else
            {
                if(baseConvexity == E_Convexity::Convex && baseBounds.u() <= 0)
                    return E_Convexity::Convex;

                if(baseConvexity == E_Convexity::Concave && baseBounds.l() >= 0)
                    return E_Convexity::Convex;

                if(baseConvexity == E_Convexity::Convex && baseBounds.l() >= 0)
                    return E_Convexity::Concave;

                if(baseConvexity == E_Convexity::Concave && baseBounds.u() <= 0)
                    return E_Convexity::Concave;
            }
        }
        else if(isInteger)
        {
            if(exponentValue > 0)
            {
                if(baseConvexity == E_Convexity::Convex && baseBounds.l() >= 0)
                    return E_Convexity::Convex;

                if(baseConvexity == E_Convexity::Concave && baseBounds.u() <= 0)
                    return E_Convexity::Concave;
            }
            else
            {
                if(baseConvexity == E_Convexity::Concave && baseBounds.l() >= 0)
                    return E_Convexity::Convex;

                if(baseConvexity == E_Convexity::Convex && baseBounds.u() <= 0)
                    return E_Convexity::Concave;
            }
        }
        else // Real exponent
        {
            if(!(baseBounds.l() >= 0))
                return E_Convexity::Unknown;

            if(baseConvexity == E_Convexity::Convex && exponentValue > 1.0)
                return E_Convexity::Convex;

            if(baseConvexity == E_Convexity::Concave && exponentValue < 0.0)
                return E_Convexity::Convex;

            if(baseConvexity == E_Convexity::Concave && exponentValue > 0.0 && exponentValue < 1.0)
                return E_Convexity::Concave;

            if(baseConvexity == E_Convexity::Convex && exponentValue < 0.0)
                return E_Convexity::Concave;
        }

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicityConstantExponent(
        double exponentValue, E_Monotonicity baseMonotonicity, Interval baseBounds) const
    {
        if(std::abs(exponentValue - 0.0) <= 1e-10 * std::abs(exponentValue))
            return E_Monotonicity::Constant;

        if(std::abs(exponentValue - 1.0) <= 1e-10 * std::abs(exponentValue))
            return baseMonotonicity;

        double intpart;
        bool isInteger = (std::modf(exponentValue, &intpart) == 0.0);
        bool integerValue = round(intpart);
        bool isEven = (integerValue % 2 == 0);

        if(isInteger && isEven)
        {
            if(exponentValue > 0)
            {
                if(baseMonotonicity == E_Monotonicity::Nondecreasing && baseBounds.l() >= 0)
                    return E_Monotonicity::Nondecreasing;

                if(baseMonotonicity == E_Monotonicity::Nonincreasing && baseBounds.u() <= 0)
                    return E_Monotonicity::Nondecreasing;

                if(baseMonotonicity == E_Monotonicity::Nondecreasing && baseBounds.u() <= 0)
                    return E_Monotonicity::Nonincreasing;

                if(baseMonotonicity == E_Monotonicity::Nonincreasing && baseBounds.l() >= 0)
                    return E_Monotonicity::Nonincreasing;
            }
            else
            {
                if(baseMonotonicity == E_Monotonicity::Nonincreasing && baseBounds.l() >= 0)
                    return E_Monotonicity::Nondecreasing;

                if(baseMonotonicity == E_Monotonicity::Nonincreasing && baseBounds.u() <= 0)
                    return E_Monotonicity::Nondecreasing;

                if(baseMonotonicity == E_Monotonicity::Nonincreasing && baseBounds.u() <= 0)
                    return E_Monotonicity::Nonincreasing;

                if(baseMonotonicity == E_Monotonicity::Nondecreasing && baseBounds.l() >= 0)
                    return E_Monotonicity::Nonincreasing;
            }
        }
        else if(isInteger)
        {
            if(exponentValue > 0 && baseMonotonicity == E_Monotonicity::Nondecreasing)
                return E_Monotonicity::Nondecreasing;

            if(exponentValue < 0 && baseMonotonicity == E_Monotonicity::Nonincreasing)
                return E_Monotonicity::Nondecreasing;

            if(exponentValue > 0 && baseMonotonicity == E_Monotonicity::Nonincreasing)
                return E_Monotonicity::Nonincreasing;

            if(exponentValue < 0 && baseMonotonicity == E_Monotonicity::Nondecreasing)
                return E_Monotonicity::Nonincreasing;
        }
        else // Real exponent
        {
            if(!(baseBounds.l() >= 0))
                return E_Monotonicity::Unknown;

            if(exponentValue > 0.0)
                return baseMonotonicity;

            if(exponentValue < 0.0)
            {
                if(baseMonotonicity == E_Monotonicity::Nondecreasing)
                    return E_Monotonicity::Nonincreasing;

                if(baseMonotonicity == E_Monotonicity::Nonincreasing)
                    return E_Monotonicity::Nondecreasing;
            }
        }

        return E_Monotonicity::Unknown;
    };

    inline E_Monotonicity getMonotonicityConstantBase(
        double baseValue, E_Monotonicity exponentMonotonicity, Interval exponentBounds) const
    {
        if(baseValue < 0.0)
            return E_Monotonicity::Unknown;

        if(std::abs(baseValue - 0.0) <= 1e-10 * std::abs(baseValue))
            return E_Monotonicity::Constant;

        if(baseValue > 0.0 && baseValue < 1.0)
        {
            if(exponentMonotonicity == E_Monotonicity::Nondecreasing && exponentBounds.u() <= 0.0)
                return E_Monotonicity::Nondecreasing;

            if(exponentMonotonicity == E_Monotonicity::Nonincreasing && exponentBounds.l() >= 0.0)
                return E_Monotonicity::Nondecreasing;
        }
        else if(baseValue >= 1.0)
            if(exponentMonotonicity == E_Monotonicity::Nondecreasing && exponentBounds.l() >= 0.0)
                return E_Monotonicity::Nondecreasing;

        if(exponentMonotonicity == E_Monotonicity::Nonincreasing && exponentBounds.u() <= 0.0)
            return E_Monotonicity::Nondecreasing;

        return E_Monotonicity::Unknown;
    };
};

// End binary operations

// Begin general operations

class ExpressionSum : public ExpressionGeneral
{
public:
    ExpressionSum() = default;

    ExpressionSum(NonlinearExpressions childExpressions) { children = childExpressions; }

    ExpressionSum(NonlinearExpressionPtr firstChild, NonlinearExpressionPtr secondChild)
    {
        NonlinearExpressions terms;
        terms.push_back(firstChild);
        terms.push_back(secondChild);
        children = terms;
    }

    inline double calculate(const VectorDouble& point) const override
    {
        double value = 0.0;

        for(auto& C : children)
        {
            value += C->calculate(point);
        }

        return (value);
    }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        Interval tmpInterval(0.);

        for(auto& C : children)
        {
            tmpInterval += C->calculate(intervalVector);
        }

        return (tmpInterval);
    }

    inline Interval getBounds() const override
    {
        Interval tmpInterval(0.);

        for(auto& C : children)
        {
            tmpInterval += C->getBounds();
        }

        return (tmpInterval);
    }

    inline FactorableFunction getFactorableFunction() override
    {
        FactorableFunction funct;

        for(auto& C : children)
        {
            funct += C->getFactorableFunction();
        }

        return (funct);
    }

    inline std::ostream& print(std::ostream& stream) const override
    {
        if(children.size() == 1)
        {
            stream << children.at(0);
            return stream;
        }

        stream << '(' << children.at(0);

        for(int i = 1; i < children.size(); i++)
        {
            stream << '+' << children.at(i);
        }

        stream << ')';

        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Sum; }

    inline E_Convexity getConvexity() const override
    {
        E_Convexity resultConvexity = E_Convexity::Linear;

        for(auto& C : children)
        {
            auto childConvexity = C->getConvexity();

            resultConvexity = Utilities::combineConvexity(resultConvexity, childConvexity);
        }

        return resultConvexity;
    };

    inline bool checkAllForConvexityType(E_Convexity convexityType)
    {
        for(auto& C : children)
        {
            if(C->getConvexity() != convexityType)
                return (false);
        }

        return (true);
    }

    inline E_Monotonicity getMonotonicity() const override
    {
        E_Monotonicity resultMonotonicity;

        bool areAllConstant = true;
        bool areAllZeroOrNondecreasing = true;
        bool areAllZeroOrNonincreasing = true;

        for(auto& C : children)
        {
            auto childMonotonicity = C->getMonotonicity();
            areAllConstant = areAllConstant && (C->getMonotonicity() == E_Monotonicity::Constant);
            areAllZeroOrNondecreasing
                = areAllZeroOrNondecreasing && (C->getMonotonicity() == E_Monotonicity::Nondecreasing);
            areAllZeroOrNonincreasing
                = areAllZeroOrNonincreasing && (C->getMonotonicity() == E_Monotonicity::Nonincreasing);
        }

        if(areAllConstant)
            return E_Monotonicity::Constant;

        if(areAllZeroOrNondecreasing)
            return E_Monotonicity::Nondecreasing;

        if(areAllZeroOrNonincreasing)
            return E_Monotonicity::Nonincreasing;

        return E_Monotonicity::Unknown;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return false;

        if(rhs.getNumberOfChildren() != getNumberOfChildren())
            return false;

        auto expression = dynamic_cast<const ExpressionSum&>(rhs);

        for(int i = 0; i < getNumberOfChildren(); i++)
        {
            if(children[i].get() != expression.children[i].get())
                return false;
        }

        return true;
    };
};

class ExpressionProduct : public ExpressionGeneral
{
public:
    ExpressionProduct() = default;

    ExpressionProduct(NonlinearExpressions childExpressions) { children = childExpressions; }

    ExpressionProduct(NonlinearExpressionPtr firstChild, NonlinearExpressionPtr secondChild)
    {
        NonlinearExpressions terms;
        terms.push_back(firstChild);
        terms.push_back(secondChild);
        children = terms;
    }

    inline double calculate(const VectorDouble& point) const override
    {
        double value = 1.0;

        for(auto& C : children)
        {
            double tmpValue = C->calculate(point);

            if(tmpValue == 0.0)
                return 0.0;

            value = value * tmpValue;
        }

        return (value);
    }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        Interval tmpInterval(1., 1.);

        for(auto& C : children)
        {
            tmpInterval = tmpInterval * C->calculate(intervalVector);
        }

        return (tmpInterval);
    }

    inline Interval getBounds() const override
    {
        Interval tmpInterval(0.);

        for(auto& C : children)
        {
            auto interval = C->getBounds();
            tmpInterval = tmpInterval * interval;
        }

        return (tmpInterval);
    }

    inline FactorableFunction getFactorableFunction() override
    {
        FactorableFunction funct;

        std::vector<FactorableFunction> factors(children.size());

        for(int i = 0; i < children.size(); i++)
        {
            factors[i] = children[i]->getFactorableFunction();
        }

        funct = factors[0];

        for(int i = 1; i < children.size(); i++)
        {
            funct = funct * factors[i];
        }

        return (funct);
    }

    inline std::ostream& print(std::ostream& stream) const override
    {
        if(children.size() == 1)
        {
            stream << children.at(0);
            return stream;
        }

        stream << '(' << children.at(0);

        for(int i = 1; i < children.size(); i++)
        {
            stream << '*' << children.at(i);
        }

        stream << ')';

        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Product; }

    inline E_Convexity getConvexity() const override
    {
        int numberOfChildren = getNumberOfChildren();

        if(numberOfChildren == 0)
            return E_Convexity::Unknown;

        if(numberOfChildren == 2)
        {
            if(children.at(0)->getType() == E_NonlinearExpressionTypes::Constant)
            {
                auto constant = std::dynamic_pointer_cast<ExpressionConstant>(children.at(0));
                auto secondConvexity = children.at(1)->getConvexity();

                if(secondConvexity == E_Convexity::Linear)
                    return E_Convexity::Linear;

                if(secondConvexity == E_Convexity::Convex && constant->constant > 0)
                    return E_Convexity::Convex;

                if(secondConvexity == E_Convexity::Convex && constant->constant < 0)
                    return E_Convexity::Concave;

                if(secondConvexity == E_Convexity::Concave && constant->constant > 0)
                    return E_Convexity::Concave;

                if(secondConvexity == E_Convexity::Concave && constant->constant < 0)
                    return E_Convexity::Convex;

                if(secondConvexity == E_Convexity::Nonconvex)
                    return E_Convexity::Nonconvex;
            }

            if(children.at(0).get() == children.at(1).get())
            {
                auto firstConvexity = children.at(0)->getConvexity();
                auto firstBounds = children.at(0)->getBounds();

                if(firstConvexity == E_Convexity::Linear)
                    return E_Convexity::Convex;

                if(firstConvexity == E_Convexity::Convex && firstBounds.l() >= 0.0)
                    return E_Convexity::Convex;

                if(firstConvexity == E_Convexity::Concave && firstBounds.u() <= 0.0)
                    return E_Convexity::Convex;
            }
        }

        return E_Convexity::Unknown;
    };

    inline E_Monotonicity getMonotonicity() const override
    {
        int numberOfChildren = getNumberOfChildren();

        if(numberOfChildren == 0)
            return E_Monotonicity::Unknown;

        E_Monotonicity joinedMonotonicity = children.at(0)->getMonotonicity();

        if(numberOfChildren == 1)
            return joinedMonotonicity;

        Interval joinedBounds = children.at(0)->getBounds();

        for(int i = 1; i < numberOfChildren; i++)
        {
            auto nextMonotonicity = children.at(i)->getMonotonicity();
            auto nextBounds = children.at(i)->getBounds();

            if(joinedMonotonicity == E_Monotonicity::Constant && nextMonotonicity == E_Monotonicity::Constant)
                return E_Monotonicity::Constant;

            if(nextMonotonicity == E_Monotonicity::Constant)
                return getMonotonicityTimesWithConstantFunction(joinedMonotonicity, joinedBounds, nextBounds);

            if(joinedMonotonicity == E_Monotonicity::Constant)
                return getMonotonicityTimesWithConstantFunction(nextMonotonicity, nextBounds, joinedBounds);

            bool nondecreasingCondition1
                = (joinedMonotonicity == E_Monotonicity::Nondecreasing && nextBounds.l() >= 0.0)
                || (joinedMonotonicity == E_Monotonicity::Nonincreasing && nextBounds.u() <= 0.0);

            bool nondecreasingCondition2
                = (nextMonotonicity == E_Monotonicity::Nondecreasing && joinedBounds.l() >= 0.0)
                || (nextMonotonicity == E_Monotonicity::Nonincreasing && joinedBounds.u() <= 0.0);

            bool nonincreasingCondition1
                = (joinedMonotonicity == E_Monotonicity::Nonincreasing && nextBounds.l() >= 0.0)
                || (joinedMonotonicity == E_Monotonicity::Nondecreasing && nextBounds.u() <= 0.0);

            bool nonincreasingCondition2
                = (nextMonotonicity == E_Monotonicity::Nonincreasing && joinedBounds.l() >= 0.0)
                || (nextMonotonicity == E_Monotonicity::Nondecreasing && joinedBounds.u() <= 0.0);

            if(nondecreasingCondition1 && nondecreasingCondition2)
                joinedMonotonicity = E_Monotonicity::Nondecreasing;
            else if(nonincreasingCondition1 && nonincreasingCondition2)
                joinedMonotonicity = E_Monotonicity::Nonincreasing;
            else
                return E_Monotonicity::Unknown;

            joinedBounds = joinedBounds * nextBounds;
        }

        return E_Monotonicity::Unknown;
    };

    inline bool operator==(const NonlinearExpression& rhs) const override
    {
        if(rhs.getType() != getType())
            return false;

        if(rhs.getNumberOfChildren() != getNumberOfChildren())
            return false;

        auto expression = dynamic_cast<const ExpressionProduct&>(rhs);

        for(int i = 0; i < getNumberOfChildren(); i++)
        {
            if(children[i].get() != expression.children[i].get())
                return false;
        }

        return true;
    };

    inline bool isLinearTerm()
    {
        assert(children.size() > 0);

        if(children.size() > 2)
            return (false);

        if(children.size() == 1)
        {
            if(children.at(0)->getType() == E_NonlinearExpressionTypes::Variable)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        // Have two children

        if(children.at(0)->getType() == E_NonlinearExpressionTypes::Constant
            && children.at(1)->getType() == E_NonlinearExpressionTypes::Variable)
        {
            return (true);
        }

        if(children.at(0)->getType() == E_NonlinearExpressionTypes::Variable
            && children.at(1)->getType() == E_NonlinearExpressionTypes::Constant)
        {
            return (true);
        }

        return (false);
    }

    inline bool isQuadraticTerm()
    {
        int powerSum = 0;

        for(auto& C : children)
        {
            if(C->getType() == E_NonlinearExpressionTypes::Square)
            {
                auto square = std::dynamic_pointer_cast<ExpressionSquare>(C);

                if(square->child->getType() != E_NonlinearExpressionTypes::Variable)
                    return (false);

                powerSum += 2;
            }
            else if(C->getType() == E_NonlinearExpressionTypes::Variable)
            {
                powerSum++;
            }
            else if(C->getType() == E_NonlinearExpressionTypes::Constant)
            {
            }
            else
            {
                return (false);
            }

            if(powerSum > 2)
                return (false);
        }

        return (powerSum == 2 ? true : false);
    }

    inline bool isMonomialTerm()
    {
        for(auto& C : children)
        {
            if(C->getType() == E_NonlinearExpressionTypes::Variable)
            {
            }
            else if(C->getType() == E_NonlinearExpressionTypes::Constant)
            {
            }
            else
            {
                return (false);
            }
        }

        return (true);
    }
};

// End general operations
} // namespace SHOT
