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

class Variable
{
public:
    std::string name = "";
    int index;

    E_VariableType type;
    std::weak_ptr<Problem> ownerProblem;

    double upperBound;
    double lowerBound;
    bool hasUpperBoundBeenTightened = false;
    bool hasLowerBoundBeenTightened = false;

    bool isNonlinear = false;
    bool isAuxiliary = false;
    FactorableFunctionPtr factorableFunctionVariable;

    Variable() : lowerBound(SHOT_DBL_MIN), upperBound(SHOT_DBL_MAX){};

    Variable(std::string variableName, int variableIndex, E_VariableType variableType, double LB, double UB)
        : name(variableName), index(variableIndex), type(variableType), lowerBound(LB), upperBound(UB){};

    Variable(std::string variableName, int variableIndex, E_VariableType variableType)
        : name(variableName)
        , index(variableIndex)
        , type(variableType)
        , lowerBound(SHOT_DBL_MIN)
        , upperBound(SHOT_DBL_MAX){};

    inline double calculate(const VectorDouble& point) { return (point[index]); }

    inline Interval calculate(const IntervalVector& intervalVector) { return intervalVector[index]; }

    inline void takeOwnership(ProblemPtr owner) { ownerProblem = owner; }
};

inline std::ostream& operator<<(std::ostream& stream, VariablePtr var)
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