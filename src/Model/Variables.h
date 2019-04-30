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

#include "ffunc.hpp"

namespace SHOT
{

typedef mc::Interval Interval;
typedef std::vector<Interval> IntervalVector;

typedef mc::FFVar FactorableFunction;
typedef std::shared_ptr<FactorableFunction> FactorableFunctionPtr;
typedef mc::Interval Interval;
typedef std::vector<Interval> IntervalVector;

class Variable
{
public:
    std::string name = "";
    int index;

    E_VariableType type;
    std::weak_ptr<Problem> ownerProblem;

    double upperBound;
    double lowerBound;
    bool hasUpperBoundBeenTightened;
    bool hasLowerBoundBeenTightened;

    bool isNonlinear;
    bool isAuxiliary;
    FactorableFunctionPtr factorableFunctionVariable;

    Variable()
        : lowerBound(SHOT_DBL_MIN)
        , upperBound(SHOT_DBL_MAX)
        , hasUpperBoundBeenTightened(false)
        , hasLowerBoundBeenTightened(false)
        , isNonlinear(false)
        , isAuxiliary(false){};

    Variable(std::string variableName, int variableIndex, E_VariableType variableType, double LB, double UB)
        : name(variableName)
        , index(variableIndex)
        , type(variableType)
        , lowerBound(LB)
        , upperBound(UB)
        , hasUpperBoundBeenTightened(false)
        , hasLowerBoundBeenTightened(false)
        , isNonlinear(false)
        , isAuxiliary(false){};

    Variable(std::string variableName, int variableIndex, E_VariableType variableType)
        : name(variableName)
        , index(variableIndex)
        , type(variableType)
        , lowerBound(SHOT_DBL_MIN)
        , upperBound(SHOT_DBL_MAX)
        , hasUpperBoundBeenTightened(false)
        , hasLowerBoundBeenTightened(false)
        , isNonlinear(false)
        , isAuxiliary(false){};

    double calculate(const VectorDouble& point) const;

    Interval calculate(const IntervalVector& intervalVector) const;
    Interval getBound();

    void takeOwnership(ProblemPtr owner);
};

typedef std::shared_ptr<Variable> VariablePtr;
typedef std::map<VariablePtr, double> SparseVariableVector;
typedef std::map<std::pair<VariablePtr, VariablePtr>, double> SparseVariableMatrix;

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

    Variables(){};

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