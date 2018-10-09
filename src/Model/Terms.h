/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "ModelShared.h"

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

    std::weak_ptr<Problem> ownerProblem;

    LinearTerm(){};
    LinearTerm(double coeff, VariablePtr var) : coefficient(coeff), variable(var){};

    double calculate(const VectorDouble &point);
    Interval calculate(const IntervalVector &intervalVector);

    void takeOwnership(ProblemPtr owner);
};

class LinearTerms
{
  public:
    std::vector<LinearTermPtr> terms;

    void add(LinearTermPtr term);

    double calculate(const VectorDouble &point);
    Interval calculate(const IntervalVector &intervalVector);

    void takeOwnership(ProblemPtr owner);
};

std::ostream &operator<<(std::ostream &stream, LinearTermPtr term);
std::ostream &operator<<(std::ostream &stream, LinearTerms linTerms);

class QuadraticTerm
{
  public:
    double coefficient;
    VariablePtr firstVariable;
    VariablePtr secondVariable;
    std::weak_ptr<Problem> ownerProblem;

    QuadraticTerm(){};
    QuadraticTerm(double coeff, VariablePtr variable1, VariablePtr variable2) : coefficient(coeff), firstVariable(variable1), secondVariable(variable2){};

    double calculate(const VectorDouble &point);

    Interval calculate(const IntervalVector &intervalVector);

    void takeOwnership(ProblemPtr owner);
};

class QuadraticTerms
{
  public:
    std::vector<QuadraticTermPtr> terms;

    void add(QuadraticTermPtr term);

    double calculate(const VectorDouble &point);
    Interval calculate(const IntervalVector &intervalVector);

    void takeOwnership(ProblemPtr owner);
};

std::ostream &operator<<(std::ostream &stream, QuadraticTermPtr term);
std::ostream &operator<<(std::ostream &stream, QuadraticTerms linTerms);

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