/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "../Enums.h"
#include "../Structs.h"

#include <algorithm>
#include <map>
#include <memory>
#include <tuple>
#include <ostream>
#include <string>

#include "interval.hpp"
#include "cppad/cppad.hpp"

namespace SHOT
{
using Interval = mc::Interval;
using IntervalVector = std::vector<Interval>;

using FactorableFunction = CppAD::AD<double>;
using FactorableFunctionPtr = std::shared_ptr<FactorableFunction>;
using Interval = mc::Interval;
using IntervalVector = std::vector<Interval>;

struct VariableProperties
{
    E_VariableType type = E_VariableType::None;
    E_AuxiliaryVariableType auxiliaryType = E_AuxiliaryVariableType::None;

    bool isAuxiliary = false;
    bool isNonlinear = false;

    bool inObjectiveFunction = false;
    bool inLinearConstraints = false;
    bool inQuadraticConstraints = false;
    bool inNonlinearConstraints = false;

    bool inLinearTerms = false;
    bool inQuadraticTerms = false;
    bool inMonomialTerms = false;
    bool inSignomialTerms = false;
    bool inNonlinearExpression = false;

    int inNumberOfLinearTerms = 0;

    bool hasUpperBoundBeenTightened = false;
    bool hasLowerBoundBeenTightened = false;

    // Whether the bound is not the one of the problem given, but the limit it has been replaced with when reading the
    // problem, e.g. Model.Variables.Integer.MinimumLowerBound for an integer variable without a lower bound
    bool hasArtificialLowerBound = false;
    bool hasArtificialUpperBound = false;

    int nonlinearVariableIndex = -1;
};

// The bounds of base^power for a constant power, which may be negative or noninteger. The interval power function
// takes the logarithm of the base, so a base reaching zero, or one with values of both signs, is handled here.
inline Interval calculateIntervalPower(Interval base, double power)
{
    if(power == 0.0)
        return (Interval(1.0));

    if(power == 1.0)
        return (base);

    double intpart;
    bool isInteger = (std::modf(power, &intpart) == 0.0);
    int integerValue = (int)round(intpart);
    bool isEven = (integerValue % 2 == 0);

    if(isInteger)
    {
        // An integer power is defined for a negative base as well, so a wholly negative domain needs no
        // adjustment at all. Only a base containing zero is a problem, and then only for a negative power,
        // where the expression grows without bound as the base approaches zero.
        if(power < 0.0 && base.l() <= 0.0 && base.u() >= 0.0)
        {
            // Only the end nearest zero is unbounded, so a domain lying on one side of zero still has a
            // bound on its other end, attained at the endpoint furthest from zero. A domain with values on
            // both sides gives a disconnected range whose hull is everything.
            if(base.l() == 0.0 && base.u() > 0.0)
                return (Interval(std::pow(base.u(), power), SHOT_DBL_MAX));

            if(base.u() == 0.0 && base.l() < 0.0)
            {
                double valueAtEndpoint = std::pow(base.l(), power);

                return (isEven ? Interval(valueAtEndpoint, SHOT_DBL_MAX) : Interval(SHOT_DBL_MIN, valueAtEndpoint));
            }

            return (Interval(SHOT_DBL_MIN, SHOT_DBL_MAX));
        }
    }
    bool baseReachesZero = false;

    if(!isInteger)
    {
        // A non-integer power has no real value for a negative base, so there is nothing to return if the
        // domain is wholly negative, and the negative part is cut away otherwise.
        if(base.u() < 0.0)
            return (Interval(SHOT_DBL_MIN, SHOT_DBL_MAX));

        // base^power grows without bound as the base approaches zero from above when the power is negative,
        // but is still bounded at the upper end of the domain. Only the non-negative part of the domain
        // contributes, so there is no real value at all if the domain does not extend above zero.
        if(power < 0.0 && base.l() <= 0.0)
        {
            if(base.u() <= 0.0)
                return (Interval(SHOT_DBL_MIN, SHOT_DBL_MAX));

            return (Interval(std::pow(base.u(), power), SHOT_DBL_MAX));
        }

        // The power is positive here, so the expression tends to zero as the base does. The base is still
        // moved off zero before evaluating, since the interval library raises to a non-integer power via a
        // logarithm and rejects a base reaching zero.
        if(base.l() <= 0.0)
        {
            baseReachesZero = true;
            base.l(SHOT_DBL_EPS);
        }
    }

    Interval bounds;

    try
    {
        bounds = isInteger ? pow(base, integerValue) : pow(base, power);
    }
    catch(const mc::Interval::Exceptions&)
    {
        return (Interval(SHOT_DBL_MIN, SHOT_DBL_MAX));
    }

    if(baseReachesZero)
        bounds.l(0.0);

    // An even integer power cannot be negative; guards against rounding in the interval library.
    if(isInteger && isEven && bounds.l() < 0.0)
        bounds.l(0.0);

    return (bounds);
}

class Variable
{
    // Only the problem a variable belongs to may number it, since the index is the variable's position in that
    // problem and everything from solution points to solver columns is addressed by it
    friend class Problem;

private:
    int index = -1;

public:
    std::string name = "";

    inline int getIndex() const { return index; }

    VariableProperties properties;

    std::weak_ptr<Problem> ownerProblem;

    // Set these through Problem::setVariableBounds or tightenBounds, which also update the bound vectors stored in
    // the problem, e.g. used when the bounds of the terms are calculated
    double upperBound;
    double lowerBound;
    double semiBound;

    FactorableFunction* factorableFunctionVariable;

    Variable()
    {
        lowerBound = SHOT_DBL_MIN;
        upperBound = SHOT_DBL_MAX;
    }

    Variable(std::string variableName, E_VariableType variableType, double LB, double UB,
        double variableSemiBound = NAN)
    {
        name = variableName;

        if(variableType == E_VariableType::Binary)
        {
            // A binary variable is restricted to [0,1], but a tighter bound given by the caller is kept, since it
            // may e.g. have been fixed to one of its bounds by bound tightening
            lowerBound = std::max(LB, 0.0);
            upperBound = std::min(UB, 1.0);
        }
        else
        {
            lowerBound = LB;
            upperBound = UB;
        }

        properties.type = variableType;
        semiBound = variableSemiBound;
    };

    Variable(std::string variableName, E_VariableType variableType)
    {
        name = variableName;

        if(variableType == E_VariableType::Binary)
        {
            lowerBound = 0;
            upperBound = 1;
        }
        else
        {
            lowerBound = SHOT_DBL_MIN;
            upperBound = SHOT_DBL_MAX;
            if(variableType == E_VariableType::Semicontinuous)
                variableType = E_VariableType::Real;
            else if(variableType == E_VariableType::Semiinteger)
                variableType = E_VariableType::Integer;
        }

        properties.type = variableType;
    };

    double calculate(const VectorDouble& point) const;

    Interval calculate(const IntervalVector& intervalVector) const;
    Interval getBound();

    bool tightenBounds(const Interval bound);

    bool isUnbounded();

    void takeOwnership(ProblemPtr owner);
};

using VariablePtr = std::shared_ptr<Variable>;

// std::shared_ptr compares on the stored address, which differs between runs, so any map keyed on variables
// needs an explicit comparator ordering on the variable index instead. Without one the entries are visited in
// an order that follows the heap layout, and anything generated while iterating -- auxiliary variables,
// constraint terms, or a sum of interval bounds -- comes out differently from one run to the next.
struct VariableIndexComparator
{
    bool operator()(const VariablePtr& firstKey, const VariablePtr& secondKey) const
    {
        return (firstKey->getIndex() < secondKey->getIndex());
    }

    bool operator()(
        const std::pair<VariablePtr, double>& firstKey, const std::pair<VariablePtr, double>& secondKey) const
    {
        if(firstKey.first->getIndex() != secondKey.first->getIndex())
            return (firstKey.first->getIndex() < secondKey.first->getIndex());

        return (firstKey.second < secondKey.second);
    }

    bool operator()(
        const std::tuple<VariablePtr, VariablePtr>& firstKey, const std::tuple<VariablePtr, VariablePtr>& secondKey) const
    {
        if(std::get<0>(firstKey)->getIndex() != std::get<0>(secondKey)->getIndex())
            return (std::get<0>(firstKey)->getIndex() < std::get<0>(secondKey)->getIndex());

        return (std::get<1>(firstKey)->getIndex() < std::get<1>(secondKey)->getIndex());
    }

    bool operator()(const std::pair<VariablePtr, VariablePtr>& firstKey,
        const std::pair<VariablePtr, VariablePtr>& secondKey) const
    {
        if(firstKey.first->getIndex() != secondKey.first->getIndex())
            return (firstKey.first->getIndex() < secondKey.first->getIndex());

        return (firstKey.second->getIndex() < secondKey.second->getIndex());
    }
};

using SparseVariableVector = std::map<VariablePtr, double, VariableIndexComparator>;
using SparseVariableMatrix = std::map<std::pair<VariablePtr, VariablePtr>, double, VariableIndexComparator>;

class Variables : private std::vector<VariablePtr>
{
protected:
    std::weak_ptr<Problem> ownerProblem;

public:
    using std::vector<VariablePtr>::operator[];

    using std::vector<VariablePtr>::at;
    using std::vector<VariablePtr>::begin;
    using std::vector<VariablePtr>::clear;
    using std::vector<VariablePtr>::end;
    using std::vector<VariablePtr>::erase;
    using std::vector<VariablePtr>::push_back;
    using std::vector<VariablePtr>::reserve;
    using std::vector<VariablePtr>::resize;
    using std::vector<VariablePtr>::size;

    Variables() = default;
    Variables(std::initializer_list<VariablePtr> variables)
    {
        for(auto& V : variables)
            (*this).push_back(V);
    };

    explicit Variables(std::vector<VariablePtr> variables) : std::vector<VariablePtr>(std::move(variables)) {};

    inline void takeOwnership(ProblemPtr owner)
    {
        ownerProblem = owner;

        for(auto& V : *this)
        {
            V->takeOwnership(owner);
        }
    }

    inline void sortByIndex()
    {
        std::sort(this->begin(), this->end(), [](const VariablePtr& variableOne, const VariablePtr& variableTwo) {
            return (variableOne->getIndex() < variableTwo->getIndex());
        });
    }
};

std::ostream& operator<<(std::ostream& stream, VariablePtr var);
} // namespace SHOT