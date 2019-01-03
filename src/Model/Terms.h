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

class LinearTerm
{
  public:
    double coefficient;
    VariablePtr variable;

    std::weak_ptr<Problem> ownerProblem;

    LinearTerm(){};
    LinearTerm(double coeff, VariablePtr var) : coefficient(coeff), variable(var){};

    inline double calculate(const VectorDouble &point)
    {
        double value = coefficient * variable->calculate(point);
        return value;
    }

    inline Interval calculate(const IntervalVector &intervalVector)
    {
        Interval value = coefficient * variable->calculate(intervalVector);
        return value;
    }

    void takeOwnership(ProblemPtr owner)
    {
        ownerProblem = owner;
    }
};

inline std::ostream &operator<<(std::ostream &stream, LinearTermPtr term)
{
    if (term->coefficient == 1.0)
    {
        stream << '+';
    }
    else if (term->coefficient == -1.0)
    {
        stream << '-';
    }
    else if (term->coefficient > 0)
    {
        stream << '+' << term->coefficient << '*';
    }
    else
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

    LinearTerms(){};

    void add(LinearTermPtr term)
    {
        terms.push_back(term);
    }

    void add(LinearTerms linearTerms)
    {
        for (auto &T : linearTerms.terms)
        {
            terms.push_back(T);
        }
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

    Interval calculate(const IntervalVector &intervalVector)
    {
        Interval value = Interval(0.0, 0.0);
        for (auto T : terms)
        {
            value += T->calculate(intervalVector);
        }

        return value;
    }

    inline void takeOwnership(ProblemPtr owner)
    {
        for (auto T : terms)
        {
            T->takeOwnership(owner);
        }
    }
};

inline std::ostream &operator<<(std::ostream &stream, LinearTerms linTerms)
{
    if (linTerms.terms.size() == 0)
        return stream;

    stream << ' ' << linTerms.terms.at(0);

    for (int i = 1; i < linTerms.terms.size(); i++)
    {
        stream << linTerms.terms.at(i);
    }

    return stream;
}

class QuadraticTerm
{
  public:
    double coefficient;
    VariablePtr firstVariable;
    VariablePtr secondVariable;

    bool isBilinear = false;
    bool isSquare = false;
    bool isBinary = false;

    std::weak_ptr<Problem> ownerProblem;

    QuadraticTerm(){};
    QuadraticTerm(double coeff, VariablePtr variable1, VariablePtr variable2) : coefficient(coeff), firstVariable(variable1), secondVariable(variable2)
    {
        if (firstVariable != secondVariable)
        {
            isBilinear = true;
        }
        else
        {
            isSquare = true;
        }

        if (firstVariable->type == E_VariableType::Binary && secondVariable->type == E_VariableType::Binary)
        {
            isBinary = true;
        }
    };

    inline double calculate(const VectorDouble &point)
    {
        double value = coefficient * firstVariable->calculate(point) * secondVariable->calculate(point);
        return value;
    };

    inline Interval calculate(const IntervalVector &intervalVector)
    {
        Interval value = coefficient * firstVariable->calculate(intervalVector) * secondVariable->calculate(intervalVector);
        return value;
    }

    inline void takeOwnership(ProblemPtr owner)
    {
        ownerProblem = owner;
    }
};

inline std::ostream &operator<<(std::ostream &stream, QuadraticTermPtr term)
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

    QuadraticTerms(){};

    inline void add(QuadraticTermPtr term)
    {
        terms.push_back(term);
    };

    void add(QuadraticTerms quadraticTerms)
    {
        for (auto &T : quadraticTerms.terms)
        {
            terms.push_back(T);
        }
    }

    inline double calculate(const VectorDouble &point)
    {
        double value = 0.0;
        for (auto T : terms)
        {
            value += T->calculate(point);
        }

        return value;
    };

    inline Interval calculate(const IntervalVector &intervalVector)
    {
        Interval value = Interval(0.0, 0.0);
        for (auto T : terms)
        {
            value += T->calculate(intervalVector);
        }

        return value;
    }

    inline void takeOwnership(ProblemPtr owner)
    {
        for (auto T : terms)
        {
            T->takeOwnership(owner);
        }
    }
};

inline std::ostream &operator<<(std::ostream &stream, QuadraticTerms quadTerms)
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