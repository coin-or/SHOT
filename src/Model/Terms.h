/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Environment.h"
#include "../Enums.h"
#include "../Structs.h"

#include "Variables.h"

#include "ffunc.hpp"

#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include "Eigen/src/SparseCore/SparseUtil.h"

#include <vector>

namespace SHOT
{

typedef mc::Interval Interval;
typedef std::vector<Interval> IntervalVector;

class Term
{
public:
    double coefficient;

    std::weak_ptr<Problem> ownerProblem;

    virtual double calculate(const VectorDouble& point) = 0;

    virtual Interval calculate(const IntervalVector& intervalVector) = 0;

    void inline takeOwnership(ProblemPtr owner) { ownerProblem = owner; }

    virtual E_Convexity getConvexity() = 0;

    virtual E_Monotonicity getMonotonicity() = 0;
};

class LinearTerm : public Term
{
private:
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

    E_Convexity getConvexity() override { return E_Convexity::Linear; };

    E_Monotonicity getMonotonicity() override
    {
        if(coefficient > 0)
            return (E_Monotonicity::Nondecreasing);
        else if(coefficient < 0)
            return (E_Monotonicity::Nonincreasing);
        else
            return (E_Monotonicity::Constant);
    };
};

typedef std::shared_ptr<LinearTerm> LinearTermPtr;

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
protected:
    E_Convexity convexity = E_Convexity::NotSet;
    E_Monotonicity monotonicity = E_Monotonicity::NotSet;

    virtual void updateConvexity() = 0;

    void updateMonotonicity()
    {
        bool areAllNonincreasing = true;
        bool areAllNondecreasing = true;

        for(auto& TERM : *this)
        {
            auto monotonicity = TERM->getMonotonicity();
            areAllNonincreasing = areAllNonincreasing
                && (monotonicity == E_Monotonicity::Nonincreasing || monotonicity == E_Monotonicity::Constant);
            areAllNondecreasing = areAllNondecreasing
                && (monotonicity == E_Monotonicity::Nondecreasing || monotonicity == E_Monotonicity::Constant);
        }

        if(areAllNonincreasing)
            monotonicity = E_Monotonicity::Nonincreasing;
        else if(areAllNondecreasing)
            monotonicity = E_Monotonicity::Nondecreasing;
        else
            monotonicity = E_Monotonicity::Unknown;
    };

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

    inline E_Convexity getConvexity()
    {
        if(convexity == E_Convexity::NotSet)
            updateConvexity();

        return (convexity);
    }

    inline E_Monotonicity getMonotonicity()
    {
        if(monotonicity == E_Monotonicity::NotSet)
            updateMonotonicity();

        return (monotonicity);
    }
};

class LinearTerms : public Terms<LinearTermPtr>
{
private:
    void updateConvexity() override { convexity = E_Convexity::Linear; };

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

    void add(LinearTermPtr term)
    {
        (*this).push_back(term);
        monotonicity = E_Monotonicity::NotSet;
    }

    void add(LinearTerms terms)
    {
        for(auto& TERM : terms)
        {
            (*this).push_back(TERM);
        }

        if(terms.size() > 0)
        {
            monotonicity = E_Monotonicity::NotSet;
        }
    }
};

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

    E_Convexity getConvexity() override
    {
        if(firstVariable == secondVariable)
        {
            if(coefficient > 0)
            {
                return (E_Convexity::Convex);
            }
            else if(coefficient < 0)
            {
                return (E_Convexity::Concave);
            }
            else
            {
                return (E_Convexity::Linear);
            }
        }

        return (E_Convexity::Nonconvex);
    }

    E_Monotonicity getMonotonicity() override
    {
        if(coefficient > 0)
        {
            return (E_Monotonicity::Nondecreasing);
        }
        else if(coefficient < 0)
        {
            return (E_Monotonicity::Nonincreasing);
        }
        else
            return (E_Monotonicity::Constant);
    };
};

typedef std::shared_ptr<QuadraticTerm> QuadraticTermPtr;

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

class QuadraticTerms : public Terms<QuadraticTermPtr>
{
private:
    void updateConvexity()
    {
        if(size() == 0)
        {
            convexity = E_Convexity::Linear;
            return;
        }

        std::vector<Eigen::Triplet<double>> elements;
        elements.reserve(2 * size());

        std::map<VariablePtr, bool> variableMap;

        bool allSquares = true;
        bool allPositive = true;
        bool allNegative = true;

        for(auto& T : (*this))
        {
            if(T->firstVariable == T->secondVariable)
            {
                variableMap.insert(std::make_pair(T->firstVariable, true));
                allPositive = allPositive && T->coefficient >= 0;
                allNegative = allNegative && T->coefficient <= 0;

                elements.push_back(
                    Eigen::Triplet<double>(T->firstVariable->index, T->firstVariable->index, T->coefficient));
            }
            else
            {
                variableMap.insert(std::make_pair(T->firstVariable, true));
                variableMap.insert(std::make_pair(T->secondVariable, true));
                allSquares = false;

                // Matrix is self adjoint, so only need lower triangular elements
                if(T->firstVariable->index > T->secondVariable->index)
                {
                    elements.push_back(Eigen::Triplet<double>(
                        T->firstVariable->index, T->secondVariable->index, 0.5 * T->coefficient));
                }
                else
                {
                    elements.push_back(Eigen::Triplet<double>(
                        T->secondVariable->index, T->firstVariable->index, 0.5 * T->coefficient));
                }
            }
        }

        if(allSquares && allPositive)
        {
            convexity = E_Convexity::Convex;
            return;
        }

        if(allSquares && allNegative)
        {
            convexity = E_Convexity::Concave;
            return;
        }

        int numberOfVariables = variableMap.size();

        Eigen::SparseMatrix<double> matrix(numberOfVariables, numberOfVariables);
        matrix.setFromTriplets(elements.begin(), elements.end());

        Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> eigenSolver(
            matrix, Eigen::DecompositionOptions::EigenvaluesOnly);

        std::cout << matrix << std::endl;

        if(eigenSolver.info() != Eigen::Success)
        {
            // std::cout << "error" << std::endl;
            convexity = E_Convexity::Unknown;
            return;
        }

        bool areAllPositiveOrZero = true;
        bool areAllNegativeOrZero = true;

        for(int i = 0; i < numberOfVariables; i++)
        {
            double eigenvalue = eigenSolver.eigenvalues().col(0)[i];

            areAllNegativeOrZero = areAllNegativeOrZero && eigenvalue < 0;
            areAllPositiveOrZero = areAllPositiveOrZero && eigenvalue > 0;

            // std::cout << "eigenvalue " << i << ":" << eigenvalue << std::endl
        }

        if(areAllPositiveOrZero)
            convexity = E_Convexity::Convex;
        else if(areAllNegativeOrZero)
            convexity = E_Convexity::Concave;
        else
            convexity = E_Convexity::Nonconvex;
    };

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

    void add(QuadraticTermPtr term)
    {
        (*this).push_back(term);
        convexity = E_Convexity::NotSet;
        monotonicity = E_Monotonicity::NotSet;
    }

    void add(QuadraticTerms terms)
    {
        for(auto& TERM : terms)
        {
            (*this).push_back(TERM);
        }

        if(terms.size() > 0)
        {
            convexity = E_Convexity::NotSet;
            monotonicity = E_Monotonicity::NotSet;
        }
    }
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

    inline E_Convexity getConvexity() override { return E_Convexity::Unknown; };

    inline E_Monotonicity getMonotonicity() override { return E_Monotonicity::Unknown; };
};

typedef std::shared_ptr<MonomialTerm> MonomialTermPtr;

inline std::ostream& operator<<(std::ostream& stream, MonomialTermPtr term)
{
    stream << term->coefficient;

    for(auto& V : term->variables)
    {
        stream << '*' << V->name;
    }

    return stream;
};

class MonomialTerms : public Terms<MonomialTermPtr>
{
private:
    void updateConvexity() override { convexity = E_Convexity::Nonconvex; };

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

    void add(MonomialTermPtr term)
    {
        (*this).push_back(term);
        convexity = E_Convexity::NotSet;
        monotonicity = E_Monotonicity::NotSet;
    }

    void add(MonomialTerms terms)
    {
        for(auto& TERM : terms)
        {
            (*this).push_back(TERM);
        }

        if(terms.size() > 0)
        {
            convexity = E_Convexity::NotSet;
            monotonicity = E_Monotonicity::NotSet;
        }
    }
};

inline std::ostream& operator<<(std::ostream& stream, LinearTerms terms)
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

inline std::ostream& operator<<(std::ostream& stream, QuadraticTerms terms)
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

inline std::ostream& operator<<(std::ostream& stream, MonomialTerms terms)
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