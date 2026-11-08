/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCapabilities.h"

#include "../Model/Problem.h"

#ifdef HAS_CBC
#include "MIPSolverCbc.h"
#endif
#ifdef HAS_CPLEX
#include "MIPSolverCplex.h"
#endif
#ifdef HAS_GUROBI
#include "MIPSolverGurobi.h"
#endif
#ifdef HAS_HIGHS
#include "MIPSolverHighs.h"
#endif

namespace SHOT
{

std::optional<MIPSolverCapabilities> getMIPSolverCapabilities(ES_MIPSolver solver)
{
    switch(solver)
    {
#ifdef HAS_CBC
    case ES_MIPSolver::Cbc:
        return MIPSolverCbc::capabilities;
#endif
#ifdef HAS_CPLEX
    case ES_MIPSolver::Cplex:
        return MIPSolverCplex::capabilities;
#endif
#ifdef HAS_GUROBI
    case ES_MIPSolver::Gurobi:
        return MIPSolverGurobi::capabilities;
#endif
#ifdef HAS_HIGHS
    case ES_MIPSolver::Highs:
        return MIPSolverHighs::capabilities;
#endif
    default:
        return std::nullopt;
    }
}

std::optional<std::string> getUnsupportedMIPFeature(const MIPSolverCapabilities& capabilities, const Problem& problem)
{
    for(size_t i = 0; i < problem.specialOrderedSets.size(); ++i)
    {
        auto type = problem.specialOrderedSets[i]->type;

        if((type == E_SOSType::One && !capabilities.SOS1) || (type == E_SOSType::Two && !capabilities.SOS2))
            return "SOS" + std::string(type == E_SOSType::One ? "1" : "2") + " set " + std::to_string(i);
    }

    for(auto type : { E_VariableType::Semicontinuous, E_VariableType::Semiinteger })
    {
        const auto& variables
            = type == E_VariableType::Semicontinuous ? problem.semicontinuousVariables : problem.semiintegerVariables;

        if(variables.size() == 0)
            continue;

        auto support = type == E_VariableType::Semicontinuous ? capabilities.semicontinuous : capabilities.semiinteger;
        std::string name = type == E_VariableType::Semicontinuous ? "semicontinuous" : "semi-integer";

        if(support == E_SemiVariableSupport::None)
            return name + " variable '" + variables.at(0)->name + "'";

        if(support != E_SemiVariableSupport::SignedInterval)
        {
            for(const auto& variable : variables)
            {
                if(variable->semiBound < 0.0)
                    return "negative branch of " + name + " variable '" + variable->name + "'";
            }
        }
    }

    return std::nullopt;
}

} // namespace SHOT
