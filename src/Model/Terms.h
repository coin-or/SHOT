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

    void takeOwnership(ProblemPtr owner) { ownerProblem = owner; }
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

class LinearTerms : private std::vector<LinearTermPtr>
{
public:
    using std::vector<LinearTermPtr>::operator[];

    using std::vector<LinearTermPtr>::at;
    using std::vector<LinearTermPtr>::begin;
    using std::vector<LinearTermPtr>::clear;
    using std::vector<LinearTermPtr>::end;
    using std::vector<LinearTermPtr>::erase;
    using std::vector<LinearTermPtr>::push_back;
    using std::vector<LinearTermPtr>::reserve;
    using std::vector<LinearTermPtr>::resize;
    using std::vector<LinearTermPtr>::size;

    LinearTerms(){};

    void add(LinearTermPtr term) { (*this).push_back(term); }

    void add(LinearTerms linearTerms)
    {
        for(auto& T : linearTerms)
        {
            (*this).push_back(T);
        }
    }

    double calculate(const VectorDouble& point)
    {
        double value = 0.0;
        for(auto T : *this)
        {
            value += T->calculate(point);
        }

        return value;
    }

    Interval calculate(const IntervalVector& intervalVector)
    {
        Interval value = Interval(0.0, 0.0);
        for(auto T : *this)
        {
            value += T->calculate(intervalVector);
        }

        return value;
    }

    inline void takeOwnership(ProblemPtr owner)
    {
        for(auto T : *this)
        {
            T->takeOwnership(owner);
        }
    }
};

inline std::ostream& operator<<(std::ostream& stream, LinearTerms linTerms)
{
    if(linTerms.size() == 0)
        return stream;

    stream << ' ' << linTerms.at(0);

    for(int i = 1; i < linTerms.size(); i++)
    {
        stream << linTerms.at(i);
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
    QuadraticTerm(double coeff, VariablePtr variable1, VariablePtr variable2)
        : coefficient(coeff), firstVariable(variable1), secondVariable(variable2)
    {
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

    inline void takeOwnership(ProblemPtr owner) { ownerProblem = owner; }

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

class QuadraticTerms : private std::vector<QuadraticTermPtr>
{
public:
    using std::vector<QuadraticTermPtr>::operator[];

    using std::vector<QuadraticTermPtr>::at;
    using std::vector<QuadraticTermPtr>::begin;
    using std::vector<QuadraticTermPtr>::clear;
    using std::vector<QuadraticTermPtr>::end;
    using std::vector<QuadraticTermPtr>::erase;
    using std::vector<QuadraticTermPtr>::push_back;
    using std::vector<QuadraticTermPtr>::reserve;
    using std::vector<QuadraticTermPtr>::resize;
    using std::vector<QuadraticTermPtr>::size;

    QuadraticTerms(){};

    inline void add(QuadraticTermPtr term) { (*this).push_back(term); };

    void add(QuadraticTerms quadraticTerms)
    {
        for(auto& T : quadraticTerms)
        {
            (*this).push_back(T);
        }
    }

    inline double calculate(const VectorDouble& point)
    {
        double value = 0.0;
        for(auto T : (*this))
        {
            value += T->calculate(point);
        }

        return value;
    };

    inline Interval calculate(const IntervalVector& intervalVector)
    {
        Interval value = Interval(0.0, 0.0);
        for(auto T : (*this))
        {
            value += T->calculate(intervalVector);
        }

        return value;
    }

    inline void takeOwnership(ProblemPtr owner)
    {
        for(auto T : (*this))
        {
            T->takeOwnership(owner);
        }
    }
};

inline std::ostream& operator<<(std::ostream& stream, QuadraticTerms quadTerms)
{
    if(quadTerms.size() == 0)
        return stream;

    if(quadTerms.at(0)->coefficient > 0)
    {
        stream << " +" << quadTerms.at(0);
    }
    else
    {
        stream << ' ' << quadTerms.at(0);
    }

    for(int i = 1; i < quadTerms.size(); i++)
    {
        stream << " +" << quadTerms.at(i);
    }

    return stream;
};

class MonomialTerm
{
public:
    double coefficient;
    Variables variables;

    bool isBilinear = false;
    bool isSquare = false;
    bool isBinary = false;

    std::weak_ptr<Problem> ownerProblem;

    MonomialTerm(){};
    MonomialTerm(double coeff, Variables variables) : coefficient(coeff), variables(variables)
    {
        isBinary = true;

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

    inline void takeOwnership(ProblemPtr owner) { ownerProblem = owner; }
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

class MonomialTerms : private std::vector<MonomialTermPtr>
{
public:
    using std::vector<MonomialTermPtr>::operator[];

    using std::vector<MonomialTermPtr>::at;
    using std::vector<MonomialTermPtr>::begin;
    using std::vector<MonomialTermPtr>::clear;
    using std::vector<MonomialTermPtr>::end;
    using std::vector<MonomialTermPtr>::erase;
    using std::vector<MonomialTermPtr>::push_back;
    using std::vector<MonomialTermPtr>::reserve;
    using std::vector<MonomialTermPtr>::resize;
    using std::vector<MonomialTermPtr>::size;

    MonomialTerms(){};

    inline void add(MonomialTermPtr term) { (*this).push_back(term); };

    void add(MonomialTerms monomialTerms)
    {
        for(auto& T : monomialTerms)
        {
            (*this).push_back(T);
        }
    }

    inline double calculate(const VectorDouble& point)
    {
        double value = 0.0;
        for(auto T : (*this))
        {
            value += T->calculate(point);
        }

        return value;
    };

    inline Interval calculate(const IntervalVector& intervalVector)
    {
        Interval value = Interval(0.0, 0.0);
        for(auto T : (*this))
        {
            value += T->calculate(intervalVector);
        }

        return value;
    }

    inline void takeOwnership(ProblemPtr owner)
    {
        for(auto T : (*this))
        {
            T->takeOwnership(owner);
        }
    }
};

inline std::ostream& operator<<(std::ostream& stream, MonomialTerms monomialTerms)
{
    if(monomialTerms.size() == 0)
        return stream;

    if(monomialTerms.at(0)->coefficient > 0)
    {
        stream << " +" << monomialTerms.at(0);
    }
    else
    {
        stream << ' ' << monomialTerms.at(0);
    }

    for(int i = 1; i < monomialTerms.size(); i++)
    {
        stream << " +" << monomialTerms.at(i);
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