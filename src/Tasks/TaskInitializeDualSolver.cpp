/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskInitializeDualSolver.h"

namespace SHOT
{

TaskInitializeDualSolver::TaskInitializeDualSolver(EnvironmentPtr envPtr, bool useLazyStrategy) : TaskBase(envPtr)
{
    auto solver = static_cast<ES_MIPSolver>(env->settings->getIntSetting("MIP.Solver", "Dual"));

    env->timing->startTimer("DualStrategy");

    if(solver != ES_MIPSolver::Cplex && solver != ES_MIPSolver::Gurobi && solver != ES_MIPSolver::Cbc)
    {
        env->output->outputError("Error in solver definition. Check option 'Dual.MIP.Solver'.");
        throw Error("Error in MIP solver definition.  Check option 'Dual.MIP.Solver'.");
    }

    bool solverSelected = false;

    if(useLazyStrategy)
    {
#ifdef HAS_CPLEX
        if(solver == ES_MIPSolver::Cplex)
        {
#ifdef HAS_CPLEX_NEW_CALLBACK
            if(env->settings->getBoolSetting("Cplex.UseNewCallbackType", "Subsolver"))
            {
                env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverCplexLazy>(env));
                env->results->usedMIPSolver = ES_MIPSolver::Cplex;
                env->output->outputDebug("Cplex with lazy callbacks selected as MIP solver.");
                solverSelected = true;
            }
            else
#endif
            {
                env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverCplexLazyOriginalCallback>(env));
                env->results->usedMIPSolver = ES_MIPSolver::Cplex;
                env->output->outputDebug("Cplex with original lazy callbacks selected as MIP solver.");
                solverSelected = true;
            }
        }
#endif

#ifdef HAS_GUROBI
        else if(solver == ES_MIPSolver::Gurobi)
        {
            env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverGurobiLazy>(env));
            env->results->usedMIPSolver = ES_MIPSolver::Gurobi;
            env->output->outputDebug("Gurobi with lazy callbacks selected as MIP solver.");
            solverSelected = true;
        }
#endif

#ifdef HAS_CBC
        if(solver == ES_MIPSolver::Cbc)
        {
            env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverOsiCbc>(env));
            env->results->usedMIPSolver = ES_MIPSolver::Cbc;
            env->output->outputDebug("Cbc selected as MIP solver.");
            solverSelected = true;
        }
#endif
    }
    else
    {

#ifdef HAS_CPLEX
        if(solver == ES_MIPSolver::Cplex)
        {
            env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverCplex>(env));
            env->results->usedMIPSolver = ES_MIPSolver::Cplex;
            env->output->outputDebug("Cplex selected as MIP solver.");
            solverSelected = true;
        }
#endif

#ifdef HAS_GUROBI
        if(solver == ES_MIPSolver::Gurobi)
        {
            env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverGurobi>(env));
            env->results->usedMIPSolver = ES_MIPSolver::Gurobi;
            env->output->outputDebug("Gurobi selected as MIP solver.");
            solverSelected = true;
        }
#endif

#ifdef HAS_CBC
        if(solver == ES_MIPSolver::Cbc)
        {
            env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverOsiCbc>(env));
            env->results->usedMIPSolver = ES_MIPSolver::Cbc;
            env->output->outputDebug("Cbc selected as MIP solver.");
            solverSelected = true;
        }
#endif
    }

    if(!solverSelected)
    {
        env->output->outputWarning(" SHOT has not been compiled with support for selected MIP solver.");

#ifdef HAS_CBC
        env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverOsiCbc>(env));
        env->results->usedMIPSolver = ES_MIPSolver::Cbc;
        solverSelected = true;
#elif HAS_GUROBI
        env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverGurobi>(env));
        env->results->usedMIPSolver = ES_MIPSolver::Gurobi;
        solverSelected = true;
#elif HAS_CPLEX
        env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverCplex>(env));
        env->results->usedMIPSolver = ES_MIPSolver::Cplex;
        solverSelected = true;
#else
        env->output->outputCritical(" SHOT has not been compiled with support for any MIP solver.");
#endif
    }

    env->timing->stopTimer("DualStrategy");
}

TaskInitializeDualSolver::~TaskInitializeDualSolver() {}

void TaskInitializeDualSolver::run() {}
std::string TaskInitializeDualSolver::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT