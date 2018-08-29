
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

class LinearTerm
{
  public:
    double coefficient;
    VariablePtr variable;

    double calculate(const VectorDouble &point)
    {
        double value = coefficient * variable->calculate(point);
        return value;
    }
};

typedef std::shared_ptr<LinearTerm> LinearTermPtr;

class LinearTerms
{
  public:
    std::vector<LinearTermPtr> terms;

    double calculate(const VectorDouble &point)
    {
        double value = 0.0;
        for (auto T : terms)
        {
            value += T->calculate(point);
        }

        return value;
    }
};

class QuadraticTerm
{
  public:
    double coefficient;
    VariablePtr firstVariable;
    VariablePtr secondVariable;

    double calculate(const VectorDouble &point)
    {
        double value = coefficient * firstVariable->calculate(point) * secondVariable->calculate(point);
        return value;
    }
};

typedef std::shared_ptr<QuadraticTerm> QuadraticTermPtr;

class QuadraticTerms
{
  public:
    std::vector<QuadraticTermPtr> terms;

    double calculate(const VectorDouble &point)
    {
        double value = 0.0;
        for (auto T : terms)
        {
            value += T->calculate(point);
        }

        return value;
    }
};

class SignomialElement
{
  public:
    VariablePtr variable;
    double power;

    double calculate(const VectorDouble &point)
    {
        double value = pow(variable->calculate(point), power);
        return value;
    }
};

typedef std::shared_ptr<SignomialElement> SignomialElementPtr;

class SignomialElements
{
  public:
    std::vector<SignomialElementPtr> elements;

    double calculate(const VectorDouble &point)
    {
        double value = 1.0;
        for (auto T : elements)
        {
            value *= T->calculate(point);
        }

        return value;
    }
};

class SignomialTerm
{
  public:
    double coefficient;
    SignomialElements signomialElements;

    double calculate(const VectorDouble &point)
    {
        double value = coefficient * signomialElements.calculate(point);

        return value;
    }
};

typedef std::shared_ptr<SignomialTerm> SignomialTermPtr;

class SignomialTerms
{
  public:
    std::vector<SignomialTermPtr> terms;

    double calculate(const VectorDouble &point)
    {
        double value = 0.0;

        for (auto T : terms)
        {
            value += T->calculate(point);
        }

        return value;
    }
};
} // namespace SHOT