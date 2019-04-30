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
#include "Terms.h"
#include "NonlinearExpressions.h"

namespace SHOT
{

class AuxiliaryVariable : public Variable
{
public:
    E_AuxiliaryVariableType auxiliaryType = E_AuxiliaryVariableType::None;
    bool isAuxiliary = true;

    // These are used to calculate the value of the variable
    double constant;
    LinearTerms linearTerms;
    QuadraticTerms quadraticTerms;
    MonomialTerms monomialTerms;
    SignomialTerms signomialTerms;
    NonlinearExpressionPtr nonlinearExpression;

    AuxiliaryVariable()
    {
        Variable::lowerBound = SHOT_DBL_MIN;
        Variable::upperBound = SHOT_DBL_MAX;
    }

    AuxiliaryVariable(std::string variableName, int variableIndex, E_VariableType variableType, double LB, double UB)
    {
        Variable::name = variableName;
        Variable::index = variableIndex;
        Variable::type = variableType;
        Variable::lowerBound = LB;
        Variable::upperBound = UB;
    }

    AuxiliaryVariable(std::string variableName, int variableIndex, E_VariableType variableType)
    {
        Variable::name = variableName;
        Variable::index = variableIndex;
        Variable::type = variableType;
        Variable::lowerBound = SHOT_DBL_MIN;
        Variable::upperBound = SHOT_DBL_MAX;
    }

    double calculateAuxiliaryValue(VectorDouble point);
};

typedef std::shared_ptr<AuxiliaryVariable> AuxiliaryVariablePtr;

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