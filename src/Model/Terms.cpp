/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "Terms.h"
#include "Problem.h"
#include "Variables.h"

namespace SHOT
{

double LinearTerm::calculate(const VectorDouble &point)
{
    double value = coefficient * variable->calculate(point);
    return value;
}

Interval LinearTerm::calculate(const IntervalVector &intervalVector)
{
    Interval value = coefficient * variable->calculate(intervalVector);
    return value;
}

void LinearTerm::takeOwnership(ProblemPtr owner)
{
    ownerProblem = owner;
}

std::ostream &operator<<(std::ostream &stream, LinearTermPtr term)
{
    if (term->coefficient != 1.0)
    {
        stream << term->coefficient << '*';
    }

    stream << term->variable->name;
    return stream;
}

void LinearTerms::add(LinearTermPtr term)
{
    terms.push_back(term);
}

double LinearTerms::calculate(const VectorDouble &point)
{
    double value = 0.0;
    for (auto T : terms)
    {
        value += T->calculate(point);
    }

    return value;
}

Interval LinearTerms::calculate(const IntervalVector &intervalVector)
{
    Interval value = Interval(0.0, 0.0);
    for (auto T : terms)
    {
        value += T->calculate(intervalVector);
    }

    return value;
}

void LinearTerms::takeOwnership(ProblemPtr owner)
{
    for (auto T : terms)
    {
        T->takeOwnership(owner);
    }
}

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

double QuadraticTerm::calculate(const VectorDouble &point)
{
    double value = coefficient * firstVariable->calculate(point) * secondVariable->calculate(point);
    return value;
};

Interval QuadraticTerm::calculate(const IntervalVector &intervalVector)
{
    Interval value = coefficient * firstVariable->calculate(intervalVector) * secondVariable->calculate(intervalVector);
    return value;
}

void QuadraticTerm::takeOwnership(ProblemPtr owner)
{
    ownerProblem = owner;
}

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

void QuadraticTerms::add(QuadraticTermPtr term)
{
    terms.push_back(term);
};

double QuadraticTerms::calculate(const VectorDouble &point)
{
    double value = 0.0;
    for (auto T : terms)
    {
        value += T->calculate(point);
    }

    return value;
};

Interval QuadraticTerms::calculate(const IntervalVector &intervalVector)
{
    Interval value = Interval(0.0, 0.0);
    for (auto T : terms)
    {
        value += T->calculate(intervalVector);
    }

    return value;
}

void QuadraticTerms::takeOwnership(ProblemPtr owner)
{
    for (auto T : terms)
    {
        T->takeOwnership(owner);
    }
}

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
}; // namespace SHOT