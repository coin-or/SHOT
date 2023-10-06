/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Variables.h"
#include "Problem.h"

#include "spdlog/fmt/fmt.h"

#include "../Environment.h"
#include "../Output.h"
#include "../Settings.h"

namespace SHOT
{

double Variable::calculate(const VectorDouble& point) const { return point[index]; }
Interval Variable::calculate(const IntervalVector& intervalVector) const { return intervalVector[index]; }
Interval Variable::getBound() { return Interval(lowerBound, upperBound); }

bool Variable::tightenBounds(const Interval bound)
{
    bool tightened = false;
    double originalLowerBound = this->lowerBound;
    double originalUpperBound = this->upperBound;

    double epsTolerance = 1e-10;

    if(bound.l() > this->lowerBound + epsTolerance && bound.l() <= this->upperBound)
    {
        tightened = true;
        this->properties.hasLowerBoundBeenTightened = true;

        if(bound.l() == 0.0 && std::signbit(bound.l()))
        {
            // Special logic for negative zero
            this->lowerBound = -bound.l();
        }
        else if(this->properties.type == E_VariableType::Binary || this->properties.type == E_VariableType::Integer
            || this->properties.type == E_VariableType::Semiinteger)
        {
            this->lowerBound = std::ceil(bound.l());
        }
        else
        {
            this->lowerBound = bound.l();
        }
    }

    if(bound.u() < this->upperBound - epsTolerance && bound.u() >= this->lowerBound)
    {
        tightened = true;
        this->properties.hasUpperBoundBeenTightened = true;

        if(bound.u() == 0.0 && std::signbit(bound.u()))
        {
            // Special logic for negative zero
            this->upperBound = -bound.u();
        }
        else if(this->properties.type == E_VariableType::Binary || this->properties.type == E_VariableType::Integer
            || this->properties.type == E_VariableType::Semiinteger)
        {
            this->upperBound = std::floor(bound.u());
        }
        else
        {
            this->upperBound = bound.u();
        }
    }

    if(tightened)
    {
        if(auto sharedOwnerProblem = ownerProblem.lock())
        {
            if(sharedOwnerProblem->env->output)
            {
                sharedOwnerProblem->env->output->outputDebug(
                    fmt::format(" Bounds tightened for variable {}:\t[{},{}] -> [{},{}].", this->name,
                        originalLowerBound, originalUpperBound, this->lowerBound, this->upperBound));
            }
        }
    }

    return tightened;
}

bool Variable::isDualUnbounded()
{
    if(properties.inLinearConstraints || properties.inQuadraticConstraints)
        return false;

    if(auto sharedOwnerProblem = ownerProblem.lock())
    {
        double minLB;
        double maxUB;

        if(sharedOwnerProblem->env->settings)
        {
            minLB = sharedOwnerProblem->env->settings->getSetting<double>(
                "Variables.Continuous.MinimumLowerBound", "Model");
            maxUB = sharedOwnerProblem->env->settings->getSetting<double>(
                "Variables.Continuous.MaximumUpperBound", "Model");
        }
        else
        {
            minLB = -1e50;
            maxUB = 1e50;
        }

        if(lowerBound > minLB && upperBound < maxUB)
            return false;
    }

    return true;
}

void Variable::takeOwnership(ProblemPtr owner) { ownerProblem = owner; }

std::ostream& operator<<(std::ostream& stream, VariablePtr var)
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

    stream << fmt::format("[{:>6d},{:<1s}] [{:<4s}] [{:<5s}]\t{:>12f}  {:1s} <= {:^16s}  <= {:1s} {:<12f}", var->index,
        type.str(), contains.str(), inTerms.str(),
        (var->properties.type == E_VariableType::Semicontinuous || var->properties.type == E_VariableType::Semiinteger)
            ? var->semiBound
            : var->lowerBound,
        var->properties.hasLowerBoundBeenTightened ? "*" : " ", var->name,
        var->properties.hasUpperBoundBeenTightened ? "*" : " ", var->upperBound);

    return stream;
}

} // namespace SHOT