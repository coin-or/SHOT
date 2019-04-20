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

    double calculateValue(VectorDouble point);
};

typedef std::shared_ptr<AuxiliaryVariable> AuxiliaryVariablePtr;
typedef std::vector<AuxiliaryVariablePtr> AuxiliaryVariables;

std::ostream& operator<<(std::ostream& stream, AuxiliaryVariablePtr var);

} // namespace SHOT