/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "../Structs.h"
#include "../Enums.h"
#include "NonlinearExpressions.h"
#include "Terms.h"
#include "Variables.h"

namespace SHOT
{

class AuxiliaryVariable : public Variable
{
public:
    // These are used to calculate the value of the variable
    double constant = 0.0;
    LinearTerms linearTerms;
    QuadraticTerms quadraticTerms;
    MonomialTerms monomialTerms;
    SignomialTerms signomialTerms;
    NonlinearExpressionPtr nonlinearExpression;

    AuxiliaryVariable()
    {
        Variable::lowerBound = SHOT_DBL_MIN;
        Variable::upperBound = SHOT_DBL_MAX;
        properties.isAuxiliary = true;
    }

    AuxiliaryVariable(std::string variableName, int variableIndex, E_VariableType variableType, double LB, double UB,
        [[maybe_unused]] double variableSemiBound = NAN)
    {
        assert(variableType != E_VariableType::Semicontinuous); // not supported yet
        assert(variableType != E_VariableType::Semiinteger); // not supported yet
        Variable::name = variableName;
        Variable::index = variableIndex;
        properties.type = variableType;
        Variable::lowerBound = LB;
        Variable::upperBound = UB;
        properties.isAuxiliary = true;
    }

    AuxiliaryVariable(std::string variableName, int variableIndex, E_VariableType variableType)
    {
        assert(variableType != E_VariableType::Semicontinuous); // not supported yet
        assert(variableType != E_VariableType::Semiinteger); // not supported yet
        Variable::name = variableName;
        Variable::index = variableIndex;
        properties.type = variableType;
        Variable::lowerBound = SHOT_DBL_MIN;
        Variable::upperBound = SHOT_DBL_MAX;
        properties.isAuxiliary = true;
    }

    double calculate(const VectorDouble& point) const;
    Interval calculate(const IntervalVector& intervalVector) const;
};

using AuxiliaryVariablePtr = std::shared_ptr<AuxiliaryVariable>;

class AuxiliaryVariables : private std::vector<AuxiliaryVariablePtr>
{
protected:
    std::weak_ptr<Problem> ownerProblem;

public:
    using std::vector<AuxiliaryVariablePtr>::operator[];

    using std::vector<AuxiliaryVariablePtr>::at;
    using std::vector<AuxiliaryVariablePtr>::begin;
    using std::vector<AuxiliaryVariablePtr>::clear;
    using std::vector<AuxiliaryVariablePtr>::end;
    using std::vector<AuxiliaryVariablePtr>::erase;
    using std::vector<AuxiliaryVariablePtr>::push_back;
    using std::vector<AuxiliaryVariablePtr>::reserve;
    using std::vector<AuxiliaryVariablePtr>::resize;
    using std::vector<AuxiliaryVariablePtr>::size;

    AuxiliaryVariables() = default;
    AuxiliaryVariables(std::initializer_list<AuxiliaryVariablePtr> variables)
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

std::ostream& operator<<(std::ostream& stream, AuxiliaryVariablePtr var);

} // namespace SHOT