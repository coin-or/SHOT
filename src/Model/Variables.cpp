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
    // A bound with an end that is not a number comes from interval arithmetic with infinite values, e.g.,
    // inf - inf, and gives no information about either end. Its other end was used before, which fixed x in [-9.8, 0]
    // to zero when the bound of x^2 in a constraint with an unbounded signomial term was [0, NaN].
    if(std::isnan(bound.l()) || std::isnan(bound.u()))
        return (false);

    bool tightened = false;
    double originalLowerBound = this->lowerBound;
    double originalUpperBound = this->upperBound;

    double epsTolerance = 1e-10;
    bool isDiscreteType = (this->properties.type == E_VariableType::Binary
        || this->properties.type == E_VariableType::Integer || this->properties.type == E_VariableType::Semiinteger);

    // The bounds from interval arithmetic have rounding errors, which can be large, e.g., the square root of a lower
    // bound of 1e-16 is 1e-8. The bounds of a discrete variable are therefore only rounded to the next integer when
    // they exceed an integer by more than a tolerance, since, e.g., 1e-8 would otherwise make the lower bound of a
    // binary variable one. Adding zero turns a negative zero into zero.
    double integerTolerance = 1e-5;
    double lowerBound = isDiscreteType ? std::ceil(bound.l() - integerTolerance) + 0.0 : bound.l();
    double upperBound = isDiscreteType ? std::floor(bound.u() + integerTolerance) + 0.0 : bound.u();

    if(lowerBound > this->lowerBound + epsTolerance && lowerBound <= this->upperBound)
    {
        tightened = true;
        this->properties.hasLowerBoundBeenTightened = true;
        this->properties.hasArtificialLowerBound = false;

        if(lowerBound == 0.0 && std::signbit(lowerBound))
        {
            // Special logic for negative zero
            this->lowerBound = -lowerBound;
        }
        else
        {
            this->lowerBound = lowerBound;
        }
    }

    if(upperBound < this->upperBound - epsTolerance && upperBound >= this->lowerBound)
    {
        tightened = true;
        this->properties.hasUpperBoundBeenTightened = true;
        this->properties.hasArtificialUpperBound = false;

        if(upperBound == 0.0 && std::signbit(upperBound))
        {
            // Special logic for negative zero
            this->upperBound = -upperBound;
        }
        else
        {
            this->upperBound = upperBound;
        }
    }

    if(tightened)
    {
        if(auto sharedOwnerProblem = ownerProblem.lock())
        {
            // Otherwise, e.g. the bounds of the terms in the problem are still calculated with the old bounds
            sharedOwnerProblem->updateVariableBoundVectors(*this);

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

bool Variable::isUnbounded()
{
    double minLB;
    double maxUB;

    if(auto sharedOwnerProblem = ownerProblem.lock(); sharedOwnerProblem && sharedOwnerProblem->env->settings)
    {
        minLB = sharedOwnerProblem->env->settings->getSetting<double>("Model.Variables.Continuous.MinimumLowerBound");
        maxUB = sharedOwnerProblem->env->settings->getSetting<double>("Model.Variables.Continuous.MaximumUpperBound");
    }
    else
    {
        minLB = -1e50;
        maxUB = 1e50;
    }

    return !(lowerBound > minLB && upperBound < maxUB);
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

    stream << fmt::format("[{:>6d},{:<1s}] [{:<4s}] [{:<5s}]\t{:>12f}  {:1s} <= {:^16s}  <= {:1s} {:<12f}",
        var->getIndex(), type.str(), contains.str(), inTerms.str(),
        (var->properties.type == E_VariableType::Semicontinuous || var->properties.type == E_VariableType::Semiinteger)
            ? var->semiBound
            : var->lowerBound,
        var->properties.hasLowerBoundBeenTightened ? "*" : " ", var->name,
        var->properties.hasUpperBoundBeenTightened ? "*" : " ", var->upperBound);

    return stream;
}

} // namespace SHOT