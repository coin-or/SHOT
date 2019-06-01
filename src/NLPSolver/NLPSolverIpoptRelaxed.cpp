/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverIpoptRelaxed.h"

#include "../Settings.h"

#include "../Model/Problem.h"

namespace SHOT
{

NLPSolverIpoptRelaxed::NLPSolverIpoptRelaxed(EnvironmentPtr envPtr, ProblemPtr source) : INLPSolver(envPtr)
{
    sourceProblem = source;

    for(auto& V : sourceProblem->allVariables)
        originalVariableType.push_back(V->properties.type);

    lowerBounds = sourceProblem->getVariableLowerBounds();
    upperBounds = sourceProblem->getVariableUpperBounds();
}

NLPSolverIpoptRelaxed::~NLPSolverIpoptRelaxed() = default;

void NLPSolverIpoptRelaxed::setSolverSpecificInitialSettings()
{
    ipoptApplication->Options()->SetNumericValue("constr_viol_tol",
        env->settings->getSetting<double>("Ipopt.ConstraintViolationTolerance", "Subsolver") + 1e-12);

    ipoptApplication->Options()->SetNumericValue(
        "tol", env->settings->getSetting<double>("Ipopt.RelativeConvergenceTolerance", "Subsolver") + 1e-12);

    ipoptApplication->Options()->SetIntegerValue(
        "max_iter", env->settings->getSetting<int>("Ipopt.MaxIterations", "Subsolver"));

    ipoptApplication->Options()->SetNumericValue(
        "max_cpu_time", env->settings->getSetting<double>("FixedInteger.TimeLimit", "Primal"));
}

VectorDouble NLPSolverIpoptRelaxed::getSolution() { return (NLPSolverIpoptBase::getSolution()); }
} // namespace SHOT