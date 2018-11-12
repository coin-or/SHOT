/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "ModelShared.h"
#include "Variables.h"

#include <math.h>
#include <vector>
#include <string>
#include <memory>
#include <iostream>

namespace SHOT
{

class NonlinearExpression
{
  public:
    std::weak_ptr<Problem> ownerProblem;

    inline void takeOwnership(ProblemPtr owner)
    {
        ownerProblem = owner;
    }

    virtual double calculate(const VectorDouble &point) = 0;
    virtual Interval calculate(const IntervalVector &intervalVector) = 0;

    virtual FactorableFunction getFactorableFunction() = 0;

    virtual std::ostream &print(std::ostream &) const = 0;

    virtual std::string getType() = 0;

    inline friend std::ostream &operator<<(std::ostream &stream, const NonlinearExpression &expr)
    {
        return expr.print(stream); // polymorphic print via reference
    };
};

std::ostream &operator<<(std::ostream &stream, NonlinearExpressionPtr expr);

class NonlinearExpressions
{
  public:
    std::vector<NonlinearExpressionPtr> expressions;

    inline void add(NonlinearExpressionPtr expression)
    {
        expressions.push_back(expression);
    };

    inline size_t size() const
    {
        return expressions.size();
    };
};

class ExpressionConstant : public NonlinearExpression
{
  public:
    double constant = 0;
    ExpressionConstant(double constant) : constant(constant){};

    inline virtual double calculate(const VectorDouble &point) override
    {
        return constant;
    };

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (Interval(constant));
    };

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return constant;
    };

    inline std::ostream &print(std::ostream &stream) const override
    {
        return stream << constant;
    };

    inline std::string getType() override
    {
        return "constant";
    };
};

class ExpressionVariable : public NonlinearExpression
{
  public:
    VariablePtr variable;

    ExpressionVariable(VariablePtr variable) : variable(variable)
    {
        variable->isNonlinear = true;
    };

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (variable->calculate(point));
    };

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (variable->calculate(intervalVector));
    };

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return *variable->factorableFunctionVariable.get();
    };

    inline std::ostream &print(std::ostream &stream) const override
    {
        return stream << variable->name;
    };

    inline std::string getType() override
    {
        return "variable";
    };
};

class ExpressionUnary : public NonlinearExpression
{
  public:
    NonlinearExpressionPtr child;

    virtual double calculate(const VectorDouble &point) = 0;
    virtual Interval calculate(const IntervalVector &intervalVector) = 0;
    virtual FactorableFunction getFactorableFunction() = 0;
    virtual std::string getType() = 0;
};

class ExpressionBinary : public NonlinearExpression
{
  public:
    NonlinearExpressionPtr firstChild;
    NonlinearExpressionPtr secondChild;

    virtual double calculate(const VectorDouble &point) = 0;
    virtual Interval calculate(const IntervalVector &intervalVector) = 0;
    virtual FactorableFunction getFactorableFunction() = 0;
    virtual std::string getType() = 0;
};

class ExpressionGeneral : public NonlinearExpression
{
  public:
    NonlinearExpressions children;

    virtual double calculate(const VectorDouble &point) = 0;
    virtual Interval calculate(const IntervalVector &intervalVector) = 0;
    virtual FactorableFunction getFactorableFunction() = 0;
    virtual std::string getType() = 0;
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

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (-child->calculate(point));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (-child->calculate(intervalVector));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (-child->getFactorableFunction());
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "(-" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "negate";
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

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (1.0 / child->calculate(point));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (1.0 / child->calculate(intervalVector));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (1 / child->getFactorableFunction());
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "1/(" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "invert";
    }
};

class ExpressionSquareRoot : public ExpressionUnary
{
  public:
    ExpressionSquareRoot()
    {
    }

    ExpressionSquareRoot(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (sqrt(child->calculate(point)));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (sqrt(child->calculate(intervalVector)));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (sqrt(child->getFactorableFunction()));
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "sqrt(" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "squareroot";
    }
};

class ExpressionLog : public ExpressionUnary
{
  public:
    ExpressionLog()
    {
    }

    ExpressionLog(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (log(child->calculate(point)));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (log(child->calculate(intervalVector)));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (log(child->getFactorableFunction()));
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "log(" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "log";
    }
};

class ExpressionExp : public ExpressionUnary
{
  public:
    ExpressionExp()
    {
    }

    ExpressionExp(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (exp(child->calculate(point)));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (exp(child->calculate(intervalVector)));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (exp(child->getFactorableFunction()));
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "exp(" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "exp";
    }
};

class ExpressionSquare : public ExpressionUnary
{
  public:
    ExpressionSquare()
    {
    }

    ExpressionSquare(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        auto value = child->calculate(point);
        return (value * value);
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        auto value = child->calculate(intervalVector);
        return (value * value);
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (child->getFactorableFunction() * child->getFactorableFunction());
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "(" << child << ")^2";
        return stream;
    }

    inline std::string getType() override
    {
        return "square";
    }
};

class ExpressionSin : public ExpressionUnary
{
  public:
    ExpressionSin()
    {
    }

    ExpressionSin(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (sin(child->calculate(point)));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (sin(child->calculate(intervalVector)));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (sin(child->getFactorableFunction()));
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "sin(" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "sin";
    }
};

class ExpressionCos : public ExpressionUnary
{
  public:
    ExpressionCos()
    {
    }

    ExpressionCos(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (cos(child->calculate(point)));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (cos(child->calculate(intervalVector)));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (cos(child->getFactorableFunction()));
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "cos(" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "cos";
    }
};

class ExpressionTan : public ExpressionUnary
{
  public:
    ExpressionTan()
    {
    }

    ExpressionTan(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (tan(child->calculate(point)));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (tan(child->calculate(intervalVector)));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (tan(child->getFactorableFunction()));
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "tan(" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "tan";
    }
};

class ExpressionArcSin : public ExpressionUnary
{
  public:
    ExpressionArcSin()
    {
    }

    ExpressionArcSin(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (asin(child->calculate(point)));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (asin(child->calculate(intervalVector)));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (asin(child->getFactorableFunction()));
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "arcsin(" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "arcsin";
    }
};

class ExpressionArcCos : public ExpressionUnary
{
  public:
    ExpressionArcCos()
    {
    }

    ExpressionArcCos(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (acos(child->calculate(point)));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (acos(child->calculate(intervalVector)));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (acos(child->getFactorableFunction()));
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "arccos(" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "arccos";
    }
};

class ExpressionArcTan : public ExpressionUnary
{
  public:
    ExpressionArcTan()
    {
    }

    ExpressionArcTan(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (atan(child->calculate(point)));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (atan(child->calculate(intervalVector)));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (atan(child->getFactorableFunction()));
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "arctan(" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "arctan";
    }
};

class ExpressionAbs : public ExpressionUnary
{
  public:
    ExpressionAbs()
    {
    }

    ExpressionAbs(NonlinearExpressionPtr childExpression)
    {
        child = childExpression;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (abs(child->calculate(point)));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (abs(child->calculate(intervalVector)));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (fabs(child->getFactorableFunction()));
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << "abs(" << child << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "abs";
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

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (firstChild->calculate(point) + secondChild->calculate(point));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (firstChild->calculate(intervalVector) + secondChild->calculate(intervalVector));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (firstChild->getFactorableFunction() + secondChild->getFactorableFunction());
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << firstChild << '+' << secondChild;
        return stream;
    }

    inline std::string getType() override
    {
        return "plus";
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

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (firstChild->calculate(point) - secondChild->calculate(point));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (firstChild->calculate(intervalVector) - secondChild->calculate(intervalVector));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (firstChild->getFactorableFunction() - secondChild->getFactorableFunction());
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << firstChild << '-' << secondChild;
        return stream;
    }

    inline std::string getType() override
    {
        return "minus";
    }
};

class ExpressionTimes : public ExpressionBinary
{
  public:
    ExpressionTimes()
    {
    }

    ExpressionTimes(NonlinearExpressionPtr childExpression1, NonlinearExpressionPtr childExpression2)
    {
        firstChild = childExpression1;
        secondChild = childExpression2;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (firstChild->calculate(point) * secondChild->calculate(point));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (firstChild->calculate(intervalVector) * secondChild->calculate(intervalVector));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (firstChild->getFactorableFunction() * secondChild->getFactorableFunction());
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << firstChild << '*' << secondChild;
        return stream;
    }

    inline std::string getType() override
    {
        return "times";
    }
};

class ExpressionDivide : public ExpressionBinary
{
  public:
    ExpressionDivide()
    {
    }

    ExpressionDivide(NonlinearExpressionPtr childExpression1, NonlinearExpressionPtr childExpression2)
    {
        firstChild = childExpression1;
        secondChild = childExpression2;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (firstChild->calculate(point) / secondChild->calculate(point));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (firstChild->calculate(intervalVector) / secondChild->calculate(intervalVector));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (firstChild->getFactorableFunction() / secondChild->getFactorableFunction());
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << firstChild << '/' << secondChild;
        return stream;
    }

    inline std::string getType() override
    {
        return "divide";
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

    inline virtual double calculate(const VectorDouble &point) override
    {
        return (pow(firstChild->calculate(point), secondChild->calculate(point)));
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        return (pow(firstChild->calculate(intervalVector), secondChild->calculate(intervalVector)));
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        return (pow(firstChild->getFactorableFunction(), secondChild->getFactorableFunction()));
    }

    inline std::ostream &print(std::ostream &stream) const override
    {
        stream << '(' << firstChild << ")^(" << secondChild << ')';
        return stream;
    }

    inline std::string getType() override
    {
        return "power";
    }
};

// End binary operations

// Begin general operations

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

    inline virtual double calculate(const VectorDouble &point) override
    {
        double value = 0.0;

        for (auto C : children.expressions)
        {
            value += C->calculate(point);
        }

        return (value);
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        Interval tmpInterval(0.);

        for (auto C : children.expressions)
        {
            tmpInterval += C->calculate(intervalVector);
        }

        return (tmpInterval);
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        FactorableFunction funct;

        for (auto C : children.expressions)
        {
            funct += C->getFactorableFunction();
        }

        return (funct);
    }

    inline std::ostream &print(std::ostream &stream) const override
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

    inline std::string getType() override
    {
        return "sum";
    }
};

class ExpressionProduct : public ExpressionGeneral
{
  public:
    ExpressionProduct()
    {
    }

    ExpressionProduct(NonlinearExpressions childExpressions)
    {
        children = childExpressions;
    }

    inline virtual double calculate(const VectorDouble &point) override
    {
        double value = 1.0;

        for (auto C : children.expressions)
        {
            double tmpValue = C->calculate(point);

            if (tmpValue == 0.0)
                return 0.0;

            value = value * tmpValue;
        }

        return (value);
    }

    inline virtual Interval calculate(const IntervalVector &intervalVector) override
    {
        Interval tmpInterval(1., 1.);

        for (auto C : children.expressions)
        {
            tmpInterval = tmpInterval * C->calculate(intervalVector);
        }

        return (tmpInterval);
    }

    inline virtual FactorableFunction getFactorableFunction() override
    {
        FactorableFunction funct;

        std::vector<FactorableFunction> factors(children.size());

        for (int i = 0; i < children.size(); i++)
        {
            factors[i] = children.expressions[i]->getFactorableFunction();
        }

        funct = factors[0];

        for (int i = 1; i < children.size(); i++)
        {
            funct = funct * factors[i];
        }

        return (funct);
    }

    inline std::ostream &print(std::ostream &stream) const override
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

    inline std::string getType() override
    {
        return "product";
    }
};

// End general operations
} // namespace SHOT