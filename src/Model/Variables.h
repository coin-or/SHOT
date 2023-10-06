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

#include <map>
#include <memory>
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
public:
    std::string name = "";
    int index;

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

    Variable(std::string variableName, int variableIndex, E_VariableType variableType, [[maybe_unused]] double LB,
        [[maybe_unused]] double UB, double variableSemiBound = NAN)
    {
        index = variableIndex;
        name = variableName;

        if(variableType == E_VariableType::Binary)
        {
            lowerBound = 0;
            upperBound = 1;
        }
        else
        {
            lowerBound = LB;
            upperBound = UB;
        }

        properties.type = variableType;
        semiBound = variableSemiBound;
    };

    Variable(std::string variableName, int variableIndex, E_VariableType variableType)
    {
        index = variableIndex;
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

    bool isDualUnbounded();

    void takeOwnership(ProblemPtr owner);
};

using VariablePtr = std::shared_ptr<Variable>;
using SparseVariableVector = std::map<VariablePtr, double>;
using SparseVariableMatrix = std::map<std::pair<VariablePtr, VariablePtr>, double>;

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
            return (variableOne->index < variableTwo->index);
        });
    }
};

std::ostream& operator<<(std::ostream& stream, VariablePtr var);
} // namespace SHOT