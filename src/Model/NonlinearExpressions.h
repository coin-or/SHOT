
/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "ModelStructs.h"

#include <math.h> /* pow */

#include <vector>
#include <string>
#include <memory>
#include <iostream>

namespace SHOT
{

class NonlinearExpression
{
  public:
    virtual double calculate(const VectorDouble &point) = 0;
    virtual std::ostream &print(std::ostream &) const = 0;

    friend std::ostream &operator<<(std::ostream &stream, const NonlinearExpression &expr)
    {
        return expr.print(stream); // polymorphic print via reference
    };
};

typedef std::shared_ptr<NonlinearExpression> NonlinearExpressionPtr;

std::ostream &operator<<(std::ostream &stream, NonlinearExpressionPtr expr)
{
    stream << *expr;
    return stream;
}

class NonlinearExpressions
{
  public:
    std::vector<NonlinearExpressionPtr> expressions;

    void add(NonlinearExpressionPtr expression)
    {
        expressions.push_back(expression);
    };

    size_t size() const
    {
        return expressions.size();
    };
};

class ExpressionVariable : public NonlinearExpression
{
  public:
    VariablePtr variable;

    ExpressionVariable(VariablePtr var) : variable(var)
    {
    }

    virtual double calculate(const VectorDouble &point) override
    {
        return (variable->calculate(point));
    }

    std::ostream &print(std::ostream &stream) const override
    {
        return stream << variable->name;
    }
};

typedef std::shared_ptr<ExpressionVariable> ExpressionVariablePtr;

class ExpressionUnary : public NonlinearExpression
{
  public:
    NonlinearExpressionPtr child;

    virtual double calculate(const VectorDouble &point) = 0;
};

class ExpressionBinary : public NonlinearExpression
{
  public:
    NonlinearExpressionPtr firstChild;
    NonlinearExpressionPtr secondChild;

    virtual double calculate(const VectorDouble &point) = 0;
};

class ExpressionGeneral : public NonlinearExpression
{
  public:
    NonlinearExpressions children;

    virtual double calculate(const VectorDouble &point) = 0;
};

// Begin unary operations

class ExpressionNegate : public ExpressionUnary
{
  public:
    ExpressionNegate()
    {
    }

    ExpressionNegate(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    virtual double calculate(const VectorDouble &point) override
    {
        return (-child->calculate(point));
    }

    std::ostream &print(std::ostream &stream) const override
    {
        stream << "(-" << child << ')';
        return stream;
    }
};

class ExpressionInvert : public ExpressionUnary
{
  public:
    ExpressionInvert()
    {
    }

    ExpressionInvert(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    virtual double calculate(const VectorDouble &point) override
    {
        return (1.0 / child->calculate(point));
    }

    std::ostream &print(std::ostream &stream) const override
    {
        stream << "1/(" << child << ')';
        return stream;
    }
};
// End unary operations

// Begin binary operations

class ExpressionPlus : public ExpressionBinary
{
  public:
    ExpressionPlus()
    {
    }

    ExpressionPlus(NonlinearExpressionPtr childExpression1, NonlinearExpressionPtr childExpression2)
    {
        firstChild = childExpression1;
        secondChild = childExpression2;
    }

    virtual double calculate(const VectorDouble &point) override
    {
        return (firstChild->calculate(point) + secondChild->calculate(point));
    }

    std::ostream &print(std::ostream &stream) const override
    {
        stream << firstChild << '+' << secondChild;
        return stream;
    }
};

class ExpressionMinus : public ExpressionBinary
{
  public:
    ExpressionMinus()
    {
    }

    ExpressionMinus(NonlinearExpressionPtr childExpression1, NonlinearExpressionPtr childExpression2)
    {
        firstChild = childExpression1;
        secondChild = childExpression2;
    }

    virtual double calculate(const VectorDouble &point) override
    {
        return (firstChild->calculate(point) - secondChild->calculate(point));
    }

    std::ostream &print(std::ostream &stream) const override
    {
        stream << firstChild << '-' << secondChild;
        return stream;
    }
};

class ExpressionPower : public ExpressionBinary
{
  public:
    ExpressionPower()
    {
    }

    ExpressionPower(NonlinearExpressionPtr childExpression1, NonlinearExpressionPtr childExpression2)
    {
        firstChild = childExpression1;
        secondChild = childExpression2;
    }

    virtual double calculate(const VectorDouble &point) override
    {
        return (pow(firstChild->calculate(point), secondChild->calculate(point)));
    }

    std::ostream &print(std::ostream &stream) const override
    {
        stream << '(' << firstChild << ")^(" << secondChild << ')';
        return stream;
    }
};

// End binary operations

// Begin general operations

class ExpressionTimes : public ExpressionGeneral
{
  public:
    ExpressionTimes()
    {
    }

    ExpressionTimes(NonlinearExpressions childExpressions)
    {
        children = childExpressions;
    }

    virtual double calculate(const VectorDouble &point) override
    {
        double value = 1.0;

        for (auto C : children.expressions)
        {
            double tmpValue = C->calculate(point);

            if (tmpValue == 0.0)
                return 0.0;

            value *= tmpValue;
        }

        return (value);
    }

    std::ostream &print(std::ostream &stream) const override
    {
        if (children.size() == 1)
        {
            stream << children.expressions.at(0);
            return stream;
        }

        stream << '(' << children.expressions.at(0);

        for (int i = 1; i < children.expressions.size(); i++)
        {
            stream << '*' << children.expressions.at(i);
        }

        stream << ')';

        return stream;
    }
};

class ExpressionSum : public ExpressionGeneral
{
  public:
    ExpressionSum()
    {
    }

    ExpressionSum(NonlinearExpressions childExpressions)
    {
        children = childExpressions;
    }

    virtual double calculate(const VectorDouble &point) override
    {
        double value = 0.0;

        for (auto C : children.expressions)
        {
            value += C->calculate(point);
        }

        return (value);
    }

    std::ostream &print(std::ostream &stream) const override
    {
        if (children.size() == 1)
        {
            stream << children.expressions.at(0);
            return stream;
        }

        stream << '(' << children.expressions.at(0);

        for (int i = 1; i < children.expressions.size(); i++)
        {
            stream << '+' << children.expressions.at(i);
        }

        stream << ')';

        return stream;
    }
};
// End binary operations
} // namespace SHOT