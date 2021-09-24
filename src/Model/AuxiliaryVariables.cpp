/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "AuxiliaryVariables.h"

#include "spdlog/fmt/fmt.h"

namespace SHOT
{
double AuxiliaryVariable::calculate(const VectorDouble& point) const
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

Interval AuxiliaryVariable::calculate(const IntervalVector& intervalVector) const
{
    Interval interval = constant;

    interval += linearTerms.calculate(intervalVector);
    interval += quadraticTerms.calculate(intervalVector);
    interval += monomialTerms.calculate(intervalVector);
    interval += signomialTerms.calculate(intervalVector);

    if(nonlinearExpression)
        interval += nonlinearExpression->calculate(intervalVector);

    return interval;
}

std::ostream& operator<<(std::ostream& stream, AuxiliaryVariablePtr var)
{
    std::stringstream type;

    switch(var->properties.type)
    {
    case(E_VariableType::Real):
        type << "C ";
        break;

    case(E_VariableType::Binary):
        type << "B ";
        break;

    case(E_VariableType::Integer):
        type << "I ";
        break;

    case(E_VariableType::Semicontinuous):
        type << "SC";
        break;

    case(E_VariableType::Semiinteger):
        type << "SI";
        break;

    default:
        type << "? ";
        break;
    }

    std::stringstream contains;

    if(var->properties.inObjectiveFunction)
        contains << "O";
    else
        contains << " ";

    if(var->properties.inLinearConstraints)
        contains << "L";
    else
        contains << " ";

    if(var->properties.inQuadraticConstraints)
        contains << "Q";
    else
        contains << " ";

    if(var->properties.inNonlinearConstraints)
        contains << "N";
    else
        contains << " ";

    std::stringstream inTerms;

    if(var->properties.inLinearTerms)
        inTerms << "L";
    else
        inTerms << " ";

    if(var->properties.inQuadraticTerms)
        inTerms << "Q";
    else
        inTerms << " ";

    if(var->properties.inMonomialTerms)
        inTerms << "M";
    else
        inTerms << " ";

    if(var->properties.inSignomialTerms)
        inTerms << "S";
    else
        inTerms << "    ";

    if(var->properties.inNonlinearExpression)
        inTerms << "N";
    else
        inTerms << " ";

    std::stringstream auxtype;

    switch(var->properties.auxiliaryType)
    {
    case E_AuxiliaryVariableType::NonlinearObjectiveFunction:
        auxtype << "nonlinear obj. aux. var.";
        break;

    case E_AuxiliaryVariableType::NonlinearExpressionPartitioning:
        auxtype << "nonlinear sum part.";
        break;

    case E_AuxiliaryVariableType::MonomialTermsPartitioning:
        auxtype << "monomial sum part.";
        break;

    case E_AuxiliaryVariableType::SignomialTermsPartitioning:
        auxtype << "signomial sum part.";
        break;

    case E_AuxiliaryVariableType::SquareTermsPartitioning:
        auxtype << "square terms part.";
        break;

    case E_AuxiliaryVariableType::ContinuousBilinear:
        auxtype << "cont. bilinear lin.";
        break;

    case E_AuxiliaryVariableType::BinaryBilinear:
        auxtype << "bin bilinear lin.";
        break;

    case E_AuxiliaryVariableType::BinaryContinuousBilinear:
        auxtype << "mixed bin./cont. bilinear lin.";
        break;

    case E_AuxiliaryVariableType::IntegerBilinear:
        auxtype << "int. bilinear lin.";
        break;

    case E_AuxiliaryVariableType::BinaryMonomial:
        auxtype << "bin. monomial lin.";
        break;

    case E_AuxiliaryVariableType::AbsoluteValue:
        auxtype << "abs. value ref.";
        break;

    case E_AuxiliaryVariableType::AntiEpigraph:
        auxtype << "anti epigraph ref.";
        break;

    case E_AuxiliaryVariableType::EigenvalueDecomposition:
        auxtype << "eigenval. decomp. ref.";
        break;

    default:
        auxtype << "unspecified aux. var.";
        break;
    }

    stream << fmt::format("[{:>6d},{:<1s}] [{:<4s}] [{:<5s}]\t{:>12f}  {:1s} <= {:^16s}  <= {:1s} {:<12f} {:>30s}",
        var->index, type.str(), contains.str(), inTerms.str(),
        (var->properties.type == E_VariableType::Semicontinuous || var->properties.type == E_VariableType::Semiinteger)
            ? var->semiBound
            : var->lowerBound,
        var->properties.hasLowerBoundBeenTightened ? "*" : " ", var->name,
        var->properties.hasUpperBoundBeenTightened ? "*" : " ", var->upperBound, auxtype.str());

    return stream;
}

} // namespace SHOT