/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "../Structs.h"

namespace SHOT
{

// Support through SHOT's adapter, including branching objects used instead of native semi-variable types.
enum class E_SemiVariableSupport
{
    None,
    NonnegativeInterval,
    SignedInterval
};

struct MIPSolverCapabilities
{
    bool convexQuadraticObjective = false;
    bool nonconvexQuadraticObjective = false;
    bool convexQuadraticConstraints = false;
    bool nonconvexQuadraticConstraints = false;

    // Supporting the objective and the constraints separately does not imply support for their combination.
    bool nonconvexObjectiveExcludesQuadraticConstraints = false;

    bool SOS1 = false;
    bool SOS2 = false;
    bool singleTree = false;
    E_SemiVariableSupport semicontinuous = E_SemiVariableSupport::None;
    E_SemiVariableSupport semiinteger = E_SemiVariableSupport::None;

    // SHOT's finite default for missing variable bounds, which can differ from the backend's native infinity.
    double unboundedVariableBound = 1e20;
};

// No solver is constructed and no license is acquired. An unavailable backend has no capability record.
std::optional<MIPSolverCapabilities> DllExport getMIPSolverCapabilities(ES_MIPSolver solver);

// Checks the discrete features that must be retained when constructing a full MIP model. Auxiliary continuous
// relaxations can intentionally omit these features and must validate their own construction instead.
std::optional<std::string> DllExport getUnsupportedMIPFeature(
    const MIPSolverCapabilities& capabilities, const Problem& problem);

} // namespace SHOT
