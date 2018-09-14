
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

class LinearTerm
{
  public:
    double coefficient;
    VariablePtr variable;

    OptimizationProblemPtr ownerProblem;

    LinearTerm(){};
    LinearTerm(double coeff, VariablePtr var) : coefficient(coeff), variable(var){};

    double calculate(const VectorDouble &point)
    {
        double value = coefficient * variable->calculate(point);
        return value;
    }

    void takeOwnership(OptimizationProblemPtr owner)
    {
        ownerProblem = owner;
    }
};

typedef std::shared_ptr<LinearTerm> LinearTermPtr;

std::ostream &operator<<(std::ostream &stream, LinearTermPtr term)
{
    if (term->coefficient != 1.0)
    {
        stream << term->coefficient << '*';
    }

    stream << term->variable->name;
    return stream;
}

class LinearTerms
{
  public:
    std::vector<LinearTermPtr> terms;

    void add(LinearTermPtr term)
    {
        terms.push_back(term);
    }

    double calculate(const VectorDouble &point)
    {
        double value = 0.0;
        for (auto T : terms)
        {
            value += T->calculate(point);
        }

        return value;
    }

    void takeOwnership(OptimizationProblemPtr owner)
    {
        for (auto T : terms)
        {
            T->takeOwnership(owner);
        }
    }
};

std::ostream &operator<<(std::ostream &stream, LinearTerms linTerms)
{
    if (linTerms.terms.size() == 0)
        return stream;

    if (linTerms.terms.at(0)->coefficient > 0)
    {
        stream << " +" << linTerms.terms.at(0);
    }
    else
    {
        stream << ' ' << linTerms.terms.at(0);
    }

    for (int i = 1; i < linTerms.terms.size(); i++)
    {
        stream << " +" << linTerms.terms.at(i);
    }

    return stream;
}

class QuadraticTerm
{
  public:
    double coefficient;
    VariablePtr firstVariable;
    VariablePtr secondVariable;
    OptimizationProblemPtr ownerProblem;

    QuadraticTerm(){};
    QuadraticTerm(double coeff, VariablePtr variable1, VariablePtr variable2) : coefficient(coeff), firstVariable(variable1), secondVariable(variable2){};

    double calculate(const VectorDouble &point)
    {
        double value = coefficient * firstVariable->calculate(point) * secondVariable->calculate(point);
        return value;
    };

    void takeOwnership(OptimizationProblemPtr owner)
    {
        ownerProblem = owner;
    }
};

typedef std::shared_ptr<QuadraticTerm> QuadraticTermPtr;

std::ostream &operator<<(std::ostream &stream, QuadraticTermPtr term)
{
    if (term->coefficient != 1.0)
    {
        stream << term->coefficient << '*';
    }

    if (term->firstVariable == term->secondVariable)
        stream << term->firstVariable->name << "^2";
    else
        stream << term->firstVariable->name << '*' << term->secondVariable->name;

    return stream;
};

class QuadraticTerms
{
  public:
    std::vector<QuadraticTermPtr> terms;

    void add(QuadraticTermPtr term)
    {
        terms.push_back(term);
    };

    double calculate(const VectorDouble &point)
    {
        double value = 0.0;
        for (auto T : terms)
        {
            value += T->calculate(point);
        }

        return value;
    };

    void takeOwnership(OptimizationProblemPtr owner)
    {
        for (auto T : terms)
        {
            T->takeOwnership(owner);
        }
    }
};

std::ostream &operator<<(std::ostream &stream, QuadraticTerms quadTerms)
{
    if (quadTerms.terms.size() == 0)
        return stream;

    if (quadTerms.terms.at(0)->coefficient > 0)
    {
        stream << " +" << quadTerms.terms.at(0);
    }
    else
    {
        stream << ' ' << quadTerms.terms.at(0);
    }

    for (int i = 1; i < quadTerms.terms.size(); i++)
    {
        stream << " +" << quadTerms.terms.at(i);
    }

    return stream;
};

/*
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

    void add(SignomialTermPtr term)
    {
        terms.push_back(term);
    }

    double calculate(const VectorDouble &point)
    {
        double value = 0.0;

        for (auto T : terms)
        {
            value += T->calculate(point);
        }

        return value;
    }
};*/
} // namespace SHOT