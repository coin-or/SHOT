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

#include "cppad/cppad.hpp"

#include <memory>
#include <optional>
#include <tuple>
#include <vector>

namespace SHOT
{

using FactorableFunction = CppAD::AD<double>;
using FactorableFunctionPtr = std::shared_ptr<FactorableFunction>;
using Interval = mc::Interval;
using IntervalVector = std::vector<Interval>;

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
}

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
    E_Monotonicity nonconstantMonotonicity, Interval constantBound)
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
}

class NonlinearExpression
{
public:
    virtual ~NonlinearExpression() = default;

    std::weak_ptr<Problem> ownerProblem;

    virtual inline void takeOwnership(ProblemPtr owner) { ownerProblem = owner; }

    virtual double calculate([[maybe_unused]] const VectorDouble& point) const = 0;
    virtual Interval calculate([[maybe_unused]] const IntervalVector& intervalVector) const = 0;
    virtual Interval getBounds() const = 0;

    virtual bool tightenBounds(Interval bound) = 0;

    virtual FactorableFunction getFactorableFunction() = 0;

    virtual std::ostream& print(std::ostream&) const = 0;

    virtual E_NonlinearExpressionTypes getType() const = 0;

    virtual E_Convexity getConvexity() const = 0;
    virtual E_Monotonicity getMonotonicity() const = 0;

    virtual int getNumberOfChildren() const = 0;

    virtual void appendNonlinearVariables([[maybe_unused]] Variables& nonlinearVariables) = 0;

    inline friend std::ostream& operator<<(std::ostream& stream, const NonlinearExpression& expr)
    {
        return expr.print(stream); // polymorphic print via reference
    };

    virtual bool operator==(const NonlinearExpression& rhs) const = 0;
};

using NonlinearExpressionPtr = std::shared_ptr<NonlinearExpression>;

inline std::ostream& operator<<(std::ostream& stream, NonlinearExpressionPtr expr)
{
    if(expr != nullptr)
    {
        stream << *expr;
    }

    return stream;
}

bool checkPerspectiveConvexity(
    NonlinearExpressionPtr expression, double linearCoefficient, VariablePtr linearVariable, double constant);

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

    NonlinearExpressions() = default;
    NonlinearExpressions(std::initializer_list<NonlinearExpressionPtr> expressions)
    {
        for(auto& E : expressions)
            (*this).push_back(E);
    };

    inline void add(NonlinearExpressionPtr expression) { (*this).push_back(expression); };
    inline void add(NonlinearExpressions expressions)
    {
        for(auto& E : expressions)
            (*this).push_back(E);
    };
};

class ExpressionConstant : public NonlinearExpression
{
public:
    double constant = 0;
    ExpressionConstant(double constant) : constant(constant) {};

    inline double calculate([[maybe_unused]] const VectorDouble& point) const override { return constant; };

    inline Interval calculate([[maybe_unused]] const IntervalVector& intervalVector) const override
    {
        return (Interval(constant));
    };

    inline Interval getBounds() const override { return Interval(constant); };

    inline bool tightenBounds([[maybe_unused]] Interval bound) override { return false; };

    inline FactorableFunction getFactorableFunction() override { return constant; };

    inline std::ostream& print(std::ostream& stream) const override { return stream << constant; };

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::Constant; };

    inline E_Convexity getConvexity() const override { return E_Convexity::Linear; };
    inline E_Monotonicity getMonotonicity() const override { return E_Monotonicity::Constant; };

    inline int getNumberOfChildren() const override { return 0; }

    inline void appendNonlinearVariables([[maybe_unused]] Variables& nonlinearVariables) override {};

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

    ExpressionVariable(VariablePtr variable) : variable(variable) {};

    inline void takeOwnership(ProblemPtr owner) override
    {
        ownerProblem = owner;
        assert(variable->ownerProblem.lock().get() == owner.get());
    }

    inline double calculate(const VectorDouble& point) const override { return (variable->calculate(point)); };

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (variable->calculate(intervalVector));
    };

    inline FactorableFunction getFactorableFunction() override { return *(variable->factorableFunctionVariable); };

    inline Interval getBounds() const override { return (variable->getBound()); };

    inline bool tightenBounds(Interval bound) override { return (variable->tightenBounds(bound)); };

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

using ExpressionVariablePtr = std::shared_ptr<ExpressionVariable>;

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

    inline bool tightenBounds(Interval bound) override { return (child->tightenBounds(-bound)); };

    inline FactorableFunction getFactorableFunction() override { return (-child->getFactorableFunction()); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "-" << child;
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

    inline Interval getBounds() const override
    {
        auto denominatorBounds = child->getBounds();

        if(denominatorBounds.l() * denominatorBounds.u() <= 0)
            return (Interval(SHOT_DBL_MIN, SHOT_DBL_MAX));

        return (1.0 / child->getBounds());
    }

    inline bool tightenBounds(Interval bound) override
    {
        if(bound.l() == 0.0 && bound.u() > SHOT_DBL_EPS)
            bound.l(SHOT_DBL_EPS);

        if(bound.u() == 0.0 && bound.l() < -SHOT_DBL_EPS)
            bound.u(-SHOT_DBL_EPS);

        if(bound.l() <= 0.0 && bound.u() >= 0.0)
            return (false);

        return (child->tightenBounds(1.0 / bound));
    };

    inline FactorableFunction getFactorableFunction() override { return (1 / child->getFactorableFunction()); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "1/" << child;
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

    inline Interval getBounds() const override
    {
        auto childBounds = child->getBounds();

        if(childBounds.l() < 0.0)
            childBounds.l(0.0);

        return (sqrt(childBounds));
    }

    inline bool tightenBounds(Interval bound) override
    {
        if(bound.l() < 0.0 && bound.u() < 0.0)
            return (false);

        auto interval = pow(bound, 2);

        return (child->tightenBounds(interval));
    };

    inline FactorableFunction getFactorableFunction() override { return (sqrt(child->getFactorableFunction())); }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << "sqrt(" << child << ')';
        return stream;
    }

    inline E_NonlinearExpressionTypes getType() const override { return E_NonlinearExpressionTypes::SquareRoot; }

    inline E_Convexity getConvexity() const override
    {
        NonlinearExpressions children;
        auto isValid = true;

        if(std::dynamic_pointer_cast<ExpressionUnary>(child) != nullptr)
        {
            children.add(std::static_pointer_cast<ExpressionUnary>(child)->child);
        }
        else if(child->getType() == E_NonlinearExpressionTypes::Sum)
        {
            children = std::dynamic_pointer_cast<ExpressionGeneral>(child)->children;
        }
        else
        {
            isValid = false;
        }

        for(auto& C : children)
        {
            if(!(C->getType() == E_NonlinearExpressionTypes::Square && C->getBounds().l() >= 0
                   && (C->getConvexity() == E_Convexity::Convex)))
            {
                isValid = false;
                break;
            }
        }

        if(isValid)
            return E_Convexity::Convex;

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
        auto childValue = child->calculate(intervalVector);

        if(childValue.l() <= 0)
            childValue.l(SHOT_DBL_EPS);

        return (log(childValue));
    }

    inline Interval getBounds() const override
    {
        auto childValue = child->getBounds();

        if(childValue.l() <= 0)
            childValue.l(SHOT_DBL_EPS);

        return (log(childValue));
    }

    inline bool tightenBounds(Interval bound) override { return (child->tightenBounds(exp(bound))); };

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

        if(childConvexity == E_Convexity::Concave || childConvexity == E_Convexity::Linear)
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

    inline bool tightenBounds(Interval bound) override
    {
        if(bound.l() <= 0)
            return false;

        return (child->tightenBounds(log(bound)));
    };

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

        auto interval = pow(value, 2);

        return (interval);
    }

    inline Interval getBounds() const override
    {
        auto value = child->getBounds();

        auto interval = pow(value, 2);

        return (interval);
    }

    inline bool tightenBounds(Interval bound) override
    {
        if(bound.l() < 0)
            return false;

        return (child->tightenBounds(sqrt(bound)));
    };

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

    inline bool tightenBounds([[maybe_unused]] Interval bound) override { return (false); };

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

    inline bool tightenBounds([[maybe_unused]] Interval bound) override { return (false); };

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

    inline bool tightenBounds([[maybe_unused]] Interval bound) override { return (false); };

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

    inline bool tightenBounds([[maybe_unused]] Interval bound) override { return (false); };

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

    inline bool tightenBounds([[maybe_unused]] Interval bound) override { return (false); };

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

    inline bool tightenBounds([[maybe_unused]] Interval bound) override { return (false); };

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

    inline double calculate(const VectorDouble& point) const override { return (fabs(child->calculate(point))); }

    inline Interval calculate(const IntervalVector& intervalVector) const override
    {
        return (fabs(child->calculate(intervalVector)));
    }

    inline Interval getBounds() const override { return (fabs(child->getBounds())); }

    inline bool tightenBounds([[maybe_unused]] Interval bound) override { return (false); };

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

    inline Interval getBounds() const override
    {
        auto denominatorBounds = secondChild->getBounds();

        if(denominatorBounds.l() * denominatorBounds.u() <= 0)
            return (Interval(SHOT_DBL_MIN, SHOT_DBL_MAX));

        return (firstChild->getBounds() / denominatorBounds);
    }

    inline bool tightenBounds([[maybe_unused]] Interval bound) override
    {
        auto bounds1 = firstChild->getBounds();
        auto bounds2 = secondChild->getBounds();

        if((bound.l() * bound.u() <= 0 || (bound.l() <= 0 && bound.u() == SHOT_DBL_INF)) && bounds1.l() >= 0
            && bounds2.l() > 0) // we know everything is positive
        {
            bound.l(SHOT_DBL_EPS);
        }
        else if((bound.l() * bound.u() <= 0 || (bound.l() == -SHOT_DBL_INF && bound.u() >= 0)) && bounds1.u() <= 0
            && bounds2.u() < 0) // we know everything is negative
        {
            bound.u(-SHOT_DBL_EPS);
        }
        else if(bound.l() * bound.u() <= 0)
        {
            return (false);
        }

        bool firstTightened = firstChild->tightenBounds(secondChild->getBounds() * bound);
        bool secondTightened = secondChild->tightenBounds(firstChild->getBounds() / bound);

        if(secondTightened)
            firstTightened = firstTightened || firstChild->tightenBounds(secondChild->getBounds() * bound);

        return (firstTightened || secondTightened);
    }

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
        auto child1Monotonicity = firstChild->getMonotonicity();
        auto child2Monotonicity = secondChild->getMonotonicity();

        auto bounds1 = firstChild->getBounds();
        auto bounds2 = secondChild->getBounds();

        if(bounds2.l() * bounds2.u() <= 0)
            return E_Convexity::Unknown;

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

        // Handle x/(c+dx), assuming c+dx > 0
        if(firstChild->getType() == E_NonlinearExpressionTypes::Variable
            && secondChild->getType() == E_NonlinearExpressionTypes::Sum && secondChild->getNumberOfChildren() == 2)
        {
            auto sum = std::dynamic_pointer_cast<ExpressionGeneral>(secondChild);
            double constant = 0.0;
            double coefficient = 0.0;
            bool isValid = false;

            ExpressionVariablePtr nominatorVariable = std::dynamic_pointer_cast<ExpressionVariable>(firstChild);
            ExpressionVariablePtr denominatorVariable;

            // x/(x+c)
            if(sum->children[0]->getType() == E_NonlinearExpressionTypes::Variable
                && sum->children[1]->getType() == E_NonlinearExpressionTypes::Constant)
            {
                denominatorVariable = std::dynamic_pointer_cast<ExpressionVariable>(sum->children[0]);
                constant = std::dynamic_pointer_cast<ExpressionConstant>(sum->children[1])->constant;
                coefficient = 1.0;
                isValid = true;
            }
            // x/(c+x)
            else if(sum->children[1]->getType() == E_NonlinearExpressionTypes::Variable
                && sum->children[0]->getType() == E_NonlinearExpressionTypes::Constant)
            {
                denominatorVariable = std::dynamic_pointer_cast<ExpressionVariable>(sum->children[0]);
                constant = std::dynamic_pointer_cast<ExpressionConstant>(sum->children[0])->constant;
                coefficient = 1.0;
                isValid = true;
            }
            // x/(d*x+c) or x/(x*d+c)
            else if(sum->children[0]->getType() == E_NonlinearExpressionTypes::Product
                && sum->children[1]->getType() == E_NonlinearExpressionTypes::Constant
                && sum->children[0]->getNumberOfChildren() == 2)
            {
                auto prod = std::dynamic_pointer_cast<ExpressionGeneral>(sum->children[0]);

                // x/(x*d+c)
                if(prod->children[0]->getType() == E_NonlinearExpressionTypes::Variable
                    && prod->children[1]->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    denominatorVariable = std::dynamic_pointer_cast<ExpressionVariable>(prod->children[0]);

                    if(denominatorVariable->variable == nominatorVariable->variable)
                    {
                        coefficient = std::dynamic_pointer_cast<ExpressionConstant>(prod->children[1])->constant;
                        constant = std::dynamic_pointer_cast<ExpressionConstant>(sum->children[1])->constant;
                        isValid = true;
                    }
                }
                // x/(d*x+c)
                else if(prod->children[1]->getType() == E_NonlinearExpressionTypes::Variable
                    && prod->children[0]->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    denominatorVariable = std::dynamic_pointer_cast<ExpressionVariable>(prod->children[1]);

                    if(denominatorVariable->variable == nominatorVariable->variable)
                    {
                        coefficient = std::dynamic_pointer_cast<ExpressionConstant>(prod->children[0])->constant;
                        constant = std::dynamic_pointer_cast<ExpressionConstant>(sum->children[1])->constant;
                        isValid = true;
                    }
                }
            }
            // x/(c+d*x) or x/(c+x*d)
            else if(sum->children[1]->getType() == E_NonlinearExpressionTypes::Product
                && sum->children[0]->getType() == E_NonlinearExpressionTypes::Constant
                && sum->children[1]->getNumberOfChildren() == 2)
            {
                auto prod = std::dynamic_pointer_cast<ExpressionGeneral>(sum->children[1]);

                // x/(c+x*d)
                if(prod->children[0]->getType() == E_NonlinearExpressionTypes::Variable
                    && prod->children[1]->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    denominatorVariable = std::dynamic_pointer_cast<ExpressionVariable>(prod->children[0]);

                    if(denominatorVariable->variable == nominatorVariable->variable)
                    {
                        coefficient = std::dynamic_pointer_cast<ExpressionConstant>(prod->children[1])->constant;
                        constant = std::dynamic_pointer_cast<ExpressionConstant>(sum->children[0])->constant;
                        isValid = true;
                    }
                }
                // x/(c+d*x)
                else if(prod->children[1]->getType() == E_NonlinearExpressionTypes::Variable
                    && prod->children[0]->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    denominatorVariable = std::dynamic_pointer_cast<ExpressionVariable>(prod->children[1]);

                    if(denominatorVariable->variable == nominatorVariable->variable)
                    {
                        coefficient = std::dynamic_pointer_cast<ExpressionConstant>(prod->children[0])->constant;
                        constant = std::dynamic_pointer_cast<ExpressionConstant>(sum->children[0])->constant;
                        isValid = true;
                    }
                }
            }

            if(isValid)
            {
                if(constant < 0)
                {
                    if(coefficient < 0 && nominatorVariable->variable->getBound().l() > -constant / coefficient)
                        return E_Convexity::Convex;
                    if(coefficient > 0 && nominatorVariable->variable->getBound().l() > -constant / coefficient)
                        return E_Convexity::Convex;

                    if(coefficient < 0 && nominatorVariable->variable->getBound().l() > -constant / coefficient)
                        return E_Convexity::Concave;
                    if(coefficient > 0 && nominatorVariable->variable->getBound().l() < -constant / coefficient)
                        return E_Convexity::Concave;
                }

                if(constant > 0)
                {
                    if(coefficient < 0 && nominatorVariable->variable->getBound().l() < -constant / coefficient)
                        return E_Convexity::Convex;
                    if(coefficient > 0 && nominatorVariable->variable->getBound().l() < -constant / coefficient)
                        return E_Convexity::Convex;

                    if(coefficient < 0 && nominatorVariable->variable->getBound().l() > -constant / coefficient)
                        return E_Convexity::Concave;
                    if(coefficient > 0 && nominatorVariable->variable->getBound().l() > -constant / coefficient)
                        return E_Convexity::Concave;
                }
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
        auto baseBounds = firstChild->calculate(intervalVector);
        auto powerBounds = secondChild->calculate(intervalVector);

        Interval bounds(0.0);

        if(secondChild->getType() == E_NonlinearExpressionTypes::Constant)
        {
            double power = powerBounds.l();

            double intpart;
            bool isInteger = (std::modf(power, &intpart) == 0.0);
            int integerValue = (int)round(intpart);
            bool isEven = (integerValue % 2 == 0);

            if(baseBounds.l() <= 0)
            {
                if(!isInteger)
                    baseBounds.l(SHOT_DBL_EPS);
                else if(isInteger && power < 0)
                    baseBounds.l(SHOT_DBL_EPS);
            }

            if(isInteger)
                bounds = pow(baseBounds, (int)power);
            else
                bounds = pow(baseBounds, power);

            if(isInteger && isEven && bounds.l() <= 0.0)
                bounds.l(0.0);

            return (bounds);
        }

        if(powerBounds.l() < 0)
        {
            if(baseBounds.l() <= 0)
                baseBounds.l(SHOT_DBL_EPS);
        }
        else if(powerBounds.l() == 0.0)
        {
            if(baseBounds.l() < 0)
                baseBounds.l(0.0);
            if(baseBounds.l() <= 0)
                baseBounds.l(SHOT_DBL_EPS);
        }

        return (pow(baseBounds, powerBounds));
    }

    inline Interval getBounds() const override
    {
        auto baseBounds = firstChild->getBounds();
        auto powerBounds = secondChild->getBounds();

        Interval bounds(0.0);

        if(secondChild->getType() == E_NonlinearExpressionTypes::Constant)
        {
            double power = powerBounds.l();

            double intpart;
            bool isInteger = (std::modf(power, &intpart) == 0.0);
            int integerValue = (int)round(intpart);
            bool isEven = (integerValue % 2 == 0);

            if(baseBounds.l() <= 0)
            {
                if(!isInteger)
                    baseBounds.l(SHOT_DBL_SIG_MIN);
                else if(isInteger && power < 0)
                    baseBounds.l(SHOT_DBL_SIG_MIN);
            }

            if(isInteger)
                bounds = pow(baseBounds, (int)power);
            else
                bounds = pow(baseBounds, power);

            if(isInteger && isEven && bounds.l() <= 0.0)
                bounds.l(0.0);

            return (bounds);
        }

        if(powerBounds.l() < 0)
        {
            if(baseBounds.l() <= 0)
                baseBounds.l(SHOT_DBL_SIG_MIN);
        }
        else if(powerBounds.l() == 0.0)
        {
            if(baseBounds.l() < 0)
                baseBounds.l(0.0);
            if(baseBounds.l() <= 0)
                baseBounds.l(SHOT_DBL_SIG_MIN);
        }

        return (pow(baseBounds, powerBounds));
    }

    inline bool tightenBounds(Interval bound) override
    {
        if(secondChild->getMonotonicity() != E_Monotonicity::Constant)
            return (false);

        double power = secondChild->getBounds().l();

        double intpart;
        bool isInteger = (std::modf(power, &intpart) == 0.0);
        int integerValue = (int)round(intpart);
        bool isEven = (integerValue % 2 == 0);

        if(isInteger && isEven && power > 0 && bound.l() <= 0.0)
            bound.l(0.0);
        else if(bound.l() <= 0.0 && bound.u() > SHOT_DBL_SIG_MIN)
            bound.l(SHOT_DBL_SIG_MIN);
        else if(bound.u() < 0)
            return (false);

        Interval interval;

        if(power == 2.0)
            interval = sqrt(bound);
        else if(power == -1.0)
        {
            interval = 1 / bound;

            if(interval.l() < 1e-10 && interval.u() > 1e-10)
                interval.l(1e-10);
        }
        else
            interval = pow(bound, 1.0 / power);

        return (firstChild->tightenBounds(interval));
    };

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
                return (CppAD::pow(firstChild->getFactorableFunction(), power));
            }
        }

        return (pow(firstChild->getFactorableFunction(), secondChild->getFactorableFunction()));
    }

    inline std::ostream& print(std::ostream& stream) const override
    {
        stream << firstChild << "^" << secondChild;
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
            if(exponentConvexity == E_Convexity::Concave || exponentConvexity == E_Convexity::Linear)
                return E_Convexity::Convex;
        }
        else if(baseValue >= 1.0)
            if(exponentConvexity == E_Convexity::Convex || exponentConvexity == E_Convexity::Linear)
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
        int integerValue = (int)round(intpart);
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
                if((baseConvexity == E_Convexity::Linear || baseConvexity == E_Convexity::Convex
                       || baseConvexity == E_Convexity::Concave)
                    && baseBounds.l() > 0)
                    return E_Convexity::Convex;

                if((baseConvexity == E_Convexity::Linear || baseConvexity == E_Convexity::Convex
                       || baseConvexity == E_Convexity::Concave)
                    && baseBounds.u() < 0)
                    return E_Convexity::Convex;
            }
        }
        else if(isInteger) // Odd exponent
        {
            if(exponentValue > 0)
            {
                if((baseConvexity == E_Convexity::Linear || baseConvexity == E_Convexity::Convex)
                    && baseBounds.l() >= 0)
                    return E_Convexity::Convex;

                if((baseConvexity == E_Convexity::Linear || baseConvexity == E_Convexity::Concave)
                    && baseBounds.u() <= 0)
                    return E_Convexity::Concave;
            }
            else
            {
                if((baseConvexity == E_Convexity::Linear || baseConvexity == E_Convexity::Convex) && baseBounds.l() > 0)
                    return E_Convexity::Convex;

                if((baseConvexity == E_Convexity::Linear || baseConvexity == E_Convexity::Convex) && baseBounds.u() < 0)
                    return E_Convexity::Concave;
            }
        }
        else // Real exponent
        {
            if(!(baseBounds.l() >= 0))
                return E_Convexity::Unknown;

            if((baseConvexity == E_Convexity::Linear || baseConvexity == E_Convexity::Convex) && exponentValue > 1.0)
                return E_Convexity::Convex;

            if((baseConvexity == E_Convexity::Linear || baseConvexity == E_Convexity::Concave) && exponentValue < 0.0)
                return E_Convexity::Convex;

            if((baseConvexity == E_Convexity::Linear || baseConvexity == E_Convexity::Concave) && exponentValue > 0.0
                && exponentValue < 1.0)
                return E_Convexity::Concave;

            if((baseConvexity == E_Convexity::Linear || baseConvexity == E_Convexity::Convex) && exponentValue < 0.0)
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
        int integerValue = (int)round(intpart);
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

    ExpressionSum(
        NonlinearExpressionPtr firstChild, NonlinearExpressionPtr secondChild, NonlinearExpressionPtr thirdChild)
    {
        NonlinearExpressions terms;
        terms.push_back(firstChild);
        terms.push_back(secondChild);
        terms.push_back(thirdChild);
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

    inline bool tightenBounds(Interval bound) override
    {
        bool tightened = false;

        for(auto& T : children)
        {
            Interval newBound = Interval(0.0);

            for(auto& T2 : children)
            {
                if(T2 == T)
                    continue;

                newBound += T2->getBounds();
            }

            Interval candidate = bound - newBound;

            tightened = T->tightenBounds(candidate) || tightened;
        }

        return (tightened);
    };

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

        for(size_t i = 1; i < children.size(); i++)
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
        try
        {
            for(auto& C : children)
            {
                if(C->getConvexity() != convexityType)
                    return (false);
            }
        }
        catch(const mc::Interval::Exceptions&)
        {
            return (false);
        }

        return (true);
    }

    inline E_Monotonicity getMonotonicity() const override
    {
        bool areAllConstant = true;
        bool areAllZeroOrNondecreasing = true;
        bool areAllZeroOrNonincreasing = true;

        for(auto& C : children)
        {
            auto childMonotonicity = C->getMonotonicity();
            areAllConstant = areAllConstant && (childMonotonicity == E_Monotonicity::Constant);
            areAllZeroOrNondecreasing
                = areAllZeroOrNondecreasing && (childMonotonicity == E_Monotonicity::Nondecreasing);
            areAllZeroOrNonincreasing
                = areAllZeroOrNonincreasing && (childMonotonicity == E_Monotonicity::Nonincreasing);
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

    std::optional<std::tuple<double, VariablePtr, double>> getLinearTermAndConstant();
};

class ExpressionProduct : public ExpressionGeneral, public std::enable_shared_from_this<ExpressionProduct>
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

    ExpressionProduct(
        NonlinearExpressionPtr firstChild, NonlinearExpressionPtr secondChild, NonlinearExpressionPtr thirdChild)
    {
        NonlinearExpressions terms;
        terms.push_back(firstChild);
        terms.push_back(secondChild);
        terms.push_back(thirdChild);
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
        Interval tmpInterval(1.);

        for(auto& C : children)
        {
            auto interval = C->getBounds();
            tmpInterval = tmpInterval * interval;
        }

        return (tmpInterval);
    }

    inline bool tightenBounds(Interval bound) override
    {
        int numberOfChildren = getNumberOfChildren();

        if(numberOfChildren == 0)
            return (false);

        if(numberOfChildren == 1)
            return (children.at(0)->tightenBounds(bound));

        bool tightened = false;

        for(auto& C1 : children)
        {
            Interval othersBound(1.0);

            for(auto& C2 : children)
            {
                if(C1 == C2)
                    continue;

                othersBound *= C2->getBounds();
            }

            // To avoid division by zero
            if(othersBound.l() <= 0 && othersBound.u() >= 0)
                continue;

            auto childBound = bound / othersBound;

            tightened = C1->tightenBounds(childBound);
        }

        return (tightened);
    };

    inline FactorableFunction getFactorableFunction() override
    {
        FactorableFunction funct;

        std::vector<FactorableFunction> factors(children.size());

        for(size_t i = 0; i < children.size(); i++)
        {
            factors[i] = children[i]->getFactorableFunction();
        }

        funct = factors[0];

        for(size_t i = 1; i < children.size(); i++)
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

        for(size_t i = 1; i < children.size(); i++)
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

        if(numberOfChildren == 1)
            return children.at(0)->getConvexity();

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

                if(secondConvexity == E_Convexity::Nonconvex || secondConvexity == E_Convexity::Unknown)
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

        // Identify convexity for exp(q*y)/x^p for q real, p>0

        bool isValid = true;
        double constant = 1.0;

        for(auto& C : children)
        {
            if(C->getType() == E_NonlinearExpressionTypes::Constant)
            {
                constant *= std::dynamic_pointer_cast<ExpressionConstant>(C)->constant;
            }
            else if(C->getType() == E_NonlinearExpressionTypes::Exp)
            {
                auto exponential = std::dynamic_pointer_cast<ExpressionExp>(C);

                if(exponential->child->getConvexity() != E_Convexity::Linear)
                {
                    isValid = false;
                    break;
                }
            }
            else if(C->getType() == E_NonlinearExpressionTypes::Power)
            {
                auto power = std::dynamic_pointer_cast<ExpressionPower>(C);

                if(power->secondChild->getType() == E_NonlinearExpressionTypes::Constant
                    && std::dynamic_pointer_cast<ExpressionConstant>(power->secondChild)->constant > 0)
                {
                    isValid = false;
                    break;
                }

                if(power->firstChild->getBounds().l() <= 0)
                {
                    isValid = false;
                    break;
                }

                if(power->firstChild->getConvexity() != E_Convexity::Linear)
                {
                    isValid = false;
                    break;
                }
            }
            else
            {
                isValid = false;
                break;
            }
        }

        if(isValid)
        {
            if(constant >= 0)
            {
                return E_Convexity::Convex;
            }
            else
            {
                return E_Convexity::Concave;
            }
        }

        // Identify convex hull reformulation

        if(children.size() == 2)
        {
            bool isConvex = true;
            bool isValid = true;

            NonlinearExpressionPtr otherFactor;
            NonlinearExpressionPtr linearFactor;

            double linearCoefficient = 0.0;
            VariablePtr linearVariable;
            double constant = 0.0;

            // Check for and get linear factor and other factor
            for(auto& C : children)
            {
                if(C->getType() == E_NonlinearExpressionTypes::Sum && C->getNumberOfChildren() == 2)
                {
                    if(linearFactor) // Double linear factor found
                    {
                        isValid = false;
                        isConvex = false;
                        break;
                    }

                    if(auto linearTermAndConstant
                        = std::dynamic_pointer_cast<ExpressionSum>(C)->getLinearTermAndConstant();
                        linearTermAndConstant)
                    {
                        linearCoefficient = std::get<0>(*linearTermAndConstant);
                        linearVariable = std::get<1>(*linearTermAndConstant);
                        constant = std::get<2>(*linearTermAndConstant);

                        linearFactor = C;
                        continue;
                    }
                }

                if(otherFactor) // Double other factor found
                {
                    isValid = false;
                    isConvex = false;
                    break;
                }

                otherFactor = C;
            }

            if(isValid && linearFactor && otherFactor)
            {
                NonlinearExpressions terms;

                if(otherFactor->getType() == E_NonlinearExpressionTypes::Sum)
                    terms = std::dynamic_pointer_cast<ExpressionSum>(otherFactor)->children;
                else
                    terms.add(otherFactor);

                for(auto& T : terms)
                {
                    isConvex = isConvex && checkPerspectiveConvexity(T, linearCoefficient, linearVariable, constant);

                    if(!isConvex)
                        break;
                }
            }

            if(isConvex)
                return (E_Convexity::Convex);
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
                return getMonotonicityTimesWithConstantFunction(joinedMonotonicity, nextBounds);

            if(joinedMonotonicity == E_Monotonicity::Constant)
                return getMonotonicityTimesWithConstantFunction(nextMonotonicity, joinedBounds);

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
            if(C->getType() == E_NonlinearExpressionTypes::Variable) { }
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

    inline std::optional<std::tuple<double, VariablePtr>> getLinearTerm()
    {
        std::optional<std::tuple<double, VariablePtr>> result;

        if(getNumberOfChildren() != 2)
            return (result);

        if(children[0]->getType() == E_NonlinearExpressionTypes::Constant
            && children[1]->getType() == E_NonlinearExpressionTypes::Variable)
        {
            auto coefficient = std::dynamic_pointer_cast<ExpressionConstant>(children[0])->constant;
            auto variable = std::dynamic_pointer_cast<ExpressionVariable>(children[1])->variable;
            result = std::make_tuple(coefficient, variable);
        }
        else if(children[1]->getType() == E_NonlinearExpressionTypes::Constant
            && children[0]->getType() == E_NonlinearExpressionTypes::Variable)
        {
            auto coefficient = std::dynamic_pointer_cast<ExpressionConstant>(children[1])->constant;
            auto variable = std::dynamic_pointer_cast<ExpressionVariable>(children[0])->variable;
            result = std::make_tuple(coefficient, variable);
        }

        return (result);
    }
};
// End general operations

bool checkPerspectiveConvexity(std::shared_ptr<ExpressionDivide> expression, double linearCoefficient,
    VariablePtr linearVariable, double constant);
bool checkPerspectiveConvexity(std::shared_ptr<ExpressionNegate> expression, double linearCoefficient,
    VariablePtr linearVariable, double constant);
bool checkPerspectiveConvexity(std::shared_ptr<ExpressionProduct> expression, double linearCoefficient,
    VariablePtr linearVariable, double constant);
bool checkPerspectiveConvexity(std::shared_ptr<ExpressionSquare> expression, double linearCoefficient,
    VariablePtr linearVariable, double constant);
bool checkPerspectiveConvexity(
    std::shared_ptr<ExpressionLog> expression, double linearCoefficient, VariablePtr linearVariable, double constant);

} // namespace SHOT
