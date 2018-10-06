/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "ModelShared.h"

namespace SHOT
{

class Variable
{
  public:
    std::string name = "";
    int index;

    E_VariableType type;
    ProblemPtr ownerProblem;

    double upperBound;
    double lowerBound;

    bool isNonlinear = false;
    FactorableFunctionPtr factorableFunctionVariable;

    Variable() : lowerBound(-std::numeric_limits<double>::infinity()),
                 upperBound(std::numeric_limits<double>::infinity()){};

    Variable(std::string variableName, int variableIndex, E_VariableType variableType, double LB, double UB) : name(variableName),
                                                                                                               index(variableIndex),
                                                                                                               type(variableType),
                                                                                                               lowerBound(LB),
                                                                                                               upperBound(UB){};

    Variable(std::string variableName, int variableIndex, E_VariableType variableType) : name(variableName),
                                                                                         index(variableIndex),
                                                                                         type(variableType),
                                                                                         lowerBound(-std::numeric_limits<double>::infinity()),
                                                                                         upperBound(std::numeric_limits<double>::infinity()){};

    inline double calculate(const VectorDouble &point)
    {
        return (point[index]);
    }

    inline Interval calculate(const IntervalVector &intervalVector)
    {
        return intervalVector[index];
    }

    inline void takeOwnership(ProblemPtr owner)
    {
        ownerProblem = owner;
    }
};

std::ostream &operator<<(std::ostream &stream, VariablePtr var);

} // namespace SHOT