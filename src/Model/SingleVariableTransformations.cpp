/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "SingleVariableTransformations.h"
#include "Problem.h"

#include "spdlog/fmt/fmt.h"

namespace SHOT
{

std::ostream& operator<<(std::ostream& stream, const SingleVariableTransformation& transformation)
{
    return transformation.print(stream); // polymorphic print via reference
}

std::ostream& operator<<(std::ostream& stream, SingleVariableTransformationPtr transformation)
{
    stream << *transformation;
    return stream;
}

std::ostream& SingleVariableTransformation::print(std::ostream& stream) const { return stream; }

std::ostream& operator<<(std::ostream& stream, SingleVariableExponentialTransformationPtr transformation)
{
    stream << *transformation;
    return stream;
}

std::ostream& SingleVariableExponentialTransformation::print(std::ostream& stream) const
{
    std::stringstream type, breakpointList;

    breakpointList << '{';

    for(const auto& BP : breakpoints)
    {
        breakpointList << '(' << BP.point << ", " << BP.value << ") ";
    }

    breakpointList << '}';

    type << transformationVariable->name << " = "
         << "log(" << originalVariable->name << ')';

    stream << fmt::format("{:<30s} {:<20s}", type.str(), breakpointList.str());

    return stream;
}

std::ostream& operator<<(std::ostream& stream, SingleVariablePowerTransformationPtr transformation)
{
    stream << *transformation;
    return stream;
}

std::ostream& SingleVariablePowerTransformation::print(std::ostream& stream) const
{
    std::stringstream type, breakpointList;

    breakpointList << '{';

    for(const auto& BP : breakpoints)
    {
        breakpointList << '(' << BP.point << ", " << BP.value << ") ";
    }

    breakpointList << '}';

    type << transformationVariable->name << " = "
         << "pow(" << originalVariable->name << ',' << 1 / power << ')';

    stream << fmt::format("{:<30s} {:<20s}", type.str(), breakpointList.str());

    return stream;
}
} // namespace SHOT