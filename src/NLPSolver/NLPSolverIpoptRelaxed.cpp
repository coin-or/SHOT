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

    updateSettings();

    ipoptProblem = new IpoptProblem(env, sourceProblem);
    ipoptApplication = new Ipopt::IpoptApplication(false);

    Ipopt::SmartPtr<Ipopt::Journal> jrnl = new IpoptJournal(envPtr, "console", Ipopt::J_ALL);
    jrnl->SetPrintLevel(Ipopt::J_DBG, Ipopt::J_NONE);
    if(!ipoptApplication->Jnlst()->AddJournal(jrnl))
        envPtr->output->outputError("        Failed to register IpoptJournal for IPOPT output.");

    setInitialSettings();

    ipoptProblem->lowerBounds = sourceProblem->getVariableLowerBounds();
    ipoptProblem->upperBounds = sourceProblem->getVariableUpperBounds();

    Ipopt::ApplicationReturnStatus ipoptStatus = ipoptApplication->Initialize();

    if(ipoptStatus != Ipopt::Solve_Succeeded)
    {
        env->output->outputError(" Error when initializing Ipopt.");
    }
}

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