/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Variables.h"
#include "ffunc.hpp"

#include "Structs.h"

namespace SHOT
{

double Variable::calculate(const VectorDouble& point) const { return point[index]; };
Interval Variable::calculate(const IntervalVector& intervalVector) const { return intervalVector[index]; };
Interval Variable::getBound() { return Interval(lowerBound, upperBound); };

void Variable::takeOwnership(ProblemPtr owner) { ownerProblem = owner; };

std::ostream& operator<<(std::ostream& stream, VariablePtr var)
{
    stream << "[" << var->index << "]:\t";

    switch(var->type)
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

    return stream;
};

} // namespace SHOT