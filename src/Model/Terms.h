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

class Term
{
public:
    double coefficient;

    std::weak_ptr<Problem> ownerProblem;

    virtual double calculate(const VectorDouble& point) = 0;

    virtual Interval calculate(const IntervalVector& intervalVector) = 0;

    void inline takeOwnership(ProblemPtr owner) { ownerProblem = owner; }
};

class LinearTerm : public Term
{
public:
    VariablePtr variable;

    LinearTerm(){};
    LinearTerm(double coeff, VariablePtr var)
    {
        coefficient = coeff;
        variable = var;
    }

    inline double calculate(const VectorDouble& point)
    {
        double value = coefficient * variable->calculate(point);
        return value;
    }

    inline Interval calculate(const IntervalVector& intervalVector)
    {
        Interval value = coefficient * variable->calculate(intervalVector);
        return value;
    }
};

inline std::ostream& operator<<(std::ostream& stream, LinearTermPtr term)
{
    if(term->coefficient == 1.0)
    {
        stream << " +";
    }
    else if(term->coefficient == -1.0)
    {
        stream << " -";
    }
    else if(term->coefficient == 0.0)
    {
        stream << " +0.0*";
    }
    else if(term->coefficient > 0)
    {
        stream << " +" << term->coefficient << '*';
    }
    else
    {
        stream << " " << term->coefficient << '*';
    }

    stream << term->variable->name;
    return stream;
}

template <class T> class Terms : private std::vector<T>
{
public:
    using std::vector<T>::operator[];

    using std::vector<T>::at;
    using std::vector<T>::begin;
    using std::vector<T>::clear;
    using std::vector<T>::end;
    using std::vector<T>::erase;
    using std::vector<T>::push_back;
    using std::vector<T>::reserve;
    using std::vector<T>::resize;
    using std::vector<T>::size;

    Terms(){};

    void add(T term) { (*this).push_back(term); }

    void add(Terms terms)
    {
        for(auto& TERM : terms)
        {
            (*this).push_back(TERM);
        }
    }

    double calculate(const VectorDouble& point)
    {
        double value = 0.0;
        for(auto& TERM : *this)
        {
            value += TERM->calculate(point);
        }

        return value;
    }

    Interval calculate(const IntervalVector& intervalVector)
    {
        Interval value = Interval(0.0, 0.0);
        for(auto& TERM : *this)
        {
            value += TERM->calculate(intervalVector);
        }

        return value;
    }

    inline void takeOwnership(ProblemPtr owner)
    {
        for(auto& TERM : *this)
        {
            TERM->takeOwnership(owner);
        }
    }
};

template <class T> inline std::ostream& operator<<(std::ostream& stream, Terms<T> terms)
{
    if(terms.size() == 0)
        return stream;

    stream << ' ' << terms.at(0);

    for(int i = 1; i < terms.size(); i++)
    {
        stream << terms.at(i);
    }

    return stream;
}

class QuadraticTerm : public Term
{
public:
    VariablePtr firstVariable;
    VariablePtr secondVariable;

    bool isBilinear;
    bool isSquare;
    bool isBinary;

    QuadraticTerm(){};
    QuadraticTerm(double coeff, VariablePtr variable1, VariablePtr variable2)
    {
        coefficient = coeff;
        firstVariable = variable1;
        secondVariable = variable2;

        if(firstVariable != secondVariable)
        {
            isBilinear = true;
        }
        else
        {
            isSquare = true;
        }

        if(firstVariable->type == E_VariableType::Binary && secondVariable->type == E_VariableType::Binary)
        {
            isBinary = true;
        }
    };

    inline double calculate(const VectorDouble& point)
    {
        double value = coefficient * firstVariable->calculate(point) * secondVariable->calculate(point);
        return value;
    };

    inline Interval calculate(const IntervalVector& intervalVector)
    {
        Interval value
            = coefficient * firstVariable->calculate(intervalVector) * secondVariable->calculate(intervalVector);
        return value;
    }

    inline bool isConvex()
    {
        if(coefficient > 0 && firstVariable == secondVariable)
        {
            return (true);
        }

        return (false);
    }
};

inline std::ostream& operator<<(std::ostream& stream, QuadraticTermPtr term)
{
    if(term->coefficient != 1.0)
    {
        stream << term->coefficient << '*';
    }

    if(term->firstVariable == term->secondVariable)
        stream << term->firstVariable->name << "^2";
    else
        stream << term->firstVariable->name << '*' << term->secondVariable->name;

    return stream;
};

class MonomialTerm : public Term
{
public:
    Variables variables;

    bool isBilinear;
    bool isSquare;
    bool isBinary;

    MonomialTerm()
    {
        isBilinear = false;
        isSquare = false;
        isBinary = false;
    };

    MonomialTerm(double coeff, Variables vars)
    {
        coefficient = coeff;
        variables = vars;

        isBilinear = false;
        isBinary = true;
        isSquare = false;

        for(auto& V : variables)
        {
            if(V->type != E_VariableType::Binary)
            {
                isBinary = false;
                break;
            }
        }
    };

    inline double calculate(const VectorDouble& point)
    {
        double value = coefficient;

        for(auto& V : variables)
        {
            value *= V->calculate(point);
        }

        return value;
    };

    inline Interval calculate(const IntervalVector& intervalVector)
    {
        Interval value(coefficient);

        for(auto& V : variables)
        {
            value *= V->calculate(intervalVector);
        }

        return value;
    }
};

inline std::ostream& operator<<(std::ostream& stream, MonomialTermPtr term)
{
    stream << term->coefficient;

    for(auto& V : term->variables)
    {
        stream << '*' << V->name;
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