
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

namespace SHOT
{

class NonlinearExpression
{
  public:
    virtual double calculate(const VectorDouble &point) = 0;
};

typedef std::shared_ptr<NonlinearExpression> NonlinearExpressionPtr;

class ExpressionUnary : NonlinearExpression
{
  public:
    NonlinearExpressionPtr child;

    virtual double calculate(const VectorDouble &point) = 0;
};

class ExpressionBinary : NonlinearExpression
{
  public:
    NonlinearExpressionPtr firstChild;
    NonlinearExpressionPtr secondChild;

    virtual double calculate(const VectorDouble &point) = 0;
};

class ExpressionGeneral : NonlinearExpression
{
  public:
    std::vector<NonlinearExpressionPtr> children;

    virtual double calculate(const VectorDouble &point) = 0;
};

// Begin unary operations

class ExpressionNegate : ExpressionUnary
{
  public:
    virtual double calculate(const VectorDouble &point)
    {
        return (-child->calculate(point));
    }
};

class ExpressionInvert : ExpressionUnary
{
  public:
    virtual double calculate(const VectorDouble &point)
    {
        return (1.0 / child->calculate(point));
    }
};

// End unary operations

// Begin binary operations

class ExpressionPlus : ExpressionBinary
{
  public:
    virtual double calculate(const VectorDouble &point)
    {
        return (firstChild->calculate(point) + secondChild->calculate(point));
    }
};

class ExpressionMinus : ExpressionBinary
{
  public:
    virtual double calculate(const VectorDouble &point)
    {
        return (firstChild->calculate(point) - secondChild->calculate(point));
    }
};

class ExpressionPower : ExpressionBinary
{
  public:
    virtual double calculate(const VectorDouble &point)
    {
        return (pow(firstChild->calculate(point), secondChild->calculate(point)));
    }
};

// End binary operations

// Begin general operations

class ExpressionTimes : ExpressionGeneral
{
  public:
    virtual double calculate(const VectorDouble &point)
    {
        double value = 1.0;

        for (auto C : children)
        {
            double tmpValue = C->calculate(point);

            if (tmpValue == 0.0)
            {
                value = 0.0;
                break;
            }

            value *= tmpValue;
        }

        return (value);
    }
};

class ExpressionSum : ExpressionGeneral
{
  public:
    virtual double calculate(const VectorDouble &point)
    {
        double value = 0.0;

        for (auto C : children)
        {
            value += C->calculate(point);
        }

        return (value);
    }
};

// End binary operations
} // namespace SHOT