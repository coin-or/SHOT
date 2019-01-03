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

class AuxilliaryVariable : public Variable
{
  public:
    E_AuxilliaryVariableType auxilliaryType = E_AuxilliaryVariableType::None;
    bool isAuxilliary = true;

    // These are used to calculate the value of the variable
    LinearTerms linearTerms;
    QuadraticTerms quadraticTerms;
    NonlinearExpressionPtr nonlinearExpression;

    AuxilliaryVariable()
    {
        Variable::lowerBound = SHOT_DBL_MIN;
        Variable::upperBound = SHOT_DBL_MAX;
    }

    AuxilliaryVariable(std::string variableName, int variableIndex, E_VariableType variableType, double LB, double UB)
    {
        Variable::name = variableName;
        Variable::index = variableIndex;
        Variable::type = variableType;
        Variable::lowerBound = LB;
        Variable::upperBound = UB;
    }

    AuxilliaryVariable(std::string variableName, int variableIndex, E_VariableType variableType)
    {
        Variable::name = variableName;
        Variable::index = variableIndex;
        Variable::type = variableType;
        Variable::lowerBound = SHOT_DBL_MIN;
        Variable::upperBound = SHOT_DBL_MAX;
    }

    inline double calculateValue(VectorDouble point)
    {
        double value = linearTerms.calculate(point);
        value += quadraticTerms.calculate(point);

        if (nonlinearExpression)
            value += nonlinearExpression->calculate(point);

        return value;
    }
};

inline std::ostream &operator<<(std::ostream &stream, AuxilliaryVariablePtr var)
{
    stream << "[" << var->index << "]:\t";

    switch (var->type)
    {
    case E_VariableType::Real:
        stream << var->lowerBound << " <= " << var->name << " <= " << var->upperBound;
        break;

    case E_VariableType::Binary:
        stream << var->name << " in {0,1}";
        break;

    case E_VariableType::Integer:
        if (var->lowerBound == 0.0 && var->upperBound == 1.0)
            stream << var->name << " in {0,1}";
        else
            stream << var->name << " in {" << var->lowerBound << ",...," << var->upperBound << "}";
        break;

    case E_VariableType::Semicontinuous:
        stream << var->name << " in {0} or " << var->lowerBound << " <= " << var->name << " <= " << var->upperBound;
        break;

    default:
        stream << var->lowerBound << " <= " << var->name << " <= " << var->upperBound;
        break;
    }

    switch (var->auxilliaryType)
    {
    case E_AuxilliaryVariableType::NonlinearObjectiveFunction:
        stream << " (objective auxilliary variable)";
        break;

    case E_AuxilliaryVariableType::NonlinearExpressionPartitioning:
        stream
            << " (partition reformulation for nonlinear sum)";
        break;

    case E_AuxilliaryVariableType::BinaryBilinear:
        stream
            << " (binary bilinear linearization)";
        break;

    default:
        stream << " (unknown auxilliary variable)";
        break;
    }

    return stream;
};

} // namespace SHOT