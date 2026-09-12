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

    int nonlinearVariableIndex = -1;
};

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