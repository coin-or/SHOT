/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "AuxiliaryVariables.h"

namespace SHOT
{

double AuxiliaryVariable::calculateAuxiliaryValue(VectorDouble point)
{
    double value = constant;

    value += linearTerms.calculate(point);
    value += quadraticTerms.calculate(point);
    value += monomialTerms.calculate(point);
    value += signomialTerms.calculate(point);

    if(nonlinearExpression)
        value += nonlinearExpression->calculate(point);

    return value;
}

std::ostream& operator<<(std::ostream& stream, AuxiliaryVariablePtr var)
{
    stream << "[" << var->index << "]:\t";

    switch(var->properties.type)
    {
    case E_VariableType::Real:
        stream << var->lowerBound << " <= " << var->name << " <= " << var->upperBound;
        break;

    case E_VariableType::Binary:
        stream << var->name << " in {0,1}";
        break;

    case E_VariableType::Integer:
        if(var->lowerBound == 0.0 && var->upperBound == 1.0)
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

    switch(var->properties.auxiliaryType)
    {
    case E_AuxiliaryVariableType::NonlinearObjectiveFunction:
        stream << " (objective auxiliary variable)";
        break;

    case E_AuxiliaryVariableType::NonlinearExpressionPartitioning:
        stream << " (partition reformulation for nonlinear sum)";
        break;

    case E_AuxiliaryVariableType::MonomialTermsPartitioning:
        stream << " (partition reformulation for monomial sum)";
        break;

    case E_AuxiliaryVariableType::SignomialTermsPartitioning:
        stream << " (partition reformulation for signomial sum)";
        break;

    case E_AuxiliaryVariableType::BinaryBilinear:
        stream << " (binary bilinear linearization)";
        break;

    default:
        stream << " (unknown auxiliary variable)";
        break;
    }

    return stream;
}

} // namespace SHOT