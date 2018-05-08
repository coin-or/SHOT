/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskInitializeDualSolver.h"

TaskInitializeDualSolver::TaskInitializeDualSolver(EnvironmentPtr envPtr, bool useLazyStrategy) : TaskBase(envPtr)
{
    auto solver = static_cast<ES_MIPSolver>(env->settings->getIntSetting("MIP.Solver", "Dual"));

    env->process->startTimer("DualStrategy");

    if (solver != ES_MIPSolver::Cplex && solver != ES_MIPSolver::Gurobi && solver != ES_MIPSolver::Cbc)
    {
        env->output->outputError("Error in solver definition. Check option 'Dual.MIP.Solver'.");
        throw new ErrorClass("Error in MIP solver definition.  Check option 'Dual.MIP.Solver'.");
    }

    if (useLazyStrategy)
    {
#ifdef HAS_CPLEX
        if (solver == ES_MIPSolver::Cplex)
        {
#ifdef HAS_CPLEX_NEW_CALLBACK
            if (env->settings->getBoolSetting("Cplex.UseNewCallbackType", "Subsolver"))
            {
                env->dualSolver = MIPSolverPtr(new MIPSolverCplexLazy(env));
                env->process->usedMIPSolver = ES_MIPSolver::Cplex;
                env->output->outputInfo("Cplex with lazy callbacks selected as MIP solver.");
            }
            else
#endif
            {
                env->dualSolver = MIPSolverPtr(new MIPSolverCplexLazyOriginalCallback(env));
                env->process->usedMIPSolver = ES_MIPSolver::Cplex;
                env->output->outputInfo("Cplex with original lazy callbacks selected as MIP solver.");
            }
        }
#endif

#ifdef HAS_GUROBI
        if (solver == ES_MIPSolver::Gurobi)
        {
            env->dualSolver = MIPSolverPtr(new MIPSolverGurobiLazy(env));
            env->process->usedMIPSolver = ES_MIPSolver::Gurobi;
            env->output->outputInfo("Gurobi with lazy callbacks selected as MIP solver.");
        }
#endif

        if (solver == ES_MIPSolver::Cbc)
        {
            env->dualSolver = MIPSolverPtr(new MIPSolverOsiCbc(env));
            env->process->usedMIPSolver = ES_MIPSolver::Cbc;
            env->output->outputInfo("Cbc selected as MIP solver.");
        }
    }
    else
    {

#ifdef HAS_CPLEX
        if (solver == ES_MIPSolver::Cplex)
        {
            env->dualSolver = MIPSolverPtr(new MIPSolverCplex(env));
            env->process->usedMIPSolver = ES_MIPSolver::Cplex;
            env->output->outputInfo("Cplex selected as MIP solver.");
        }
#endif

#ifdef HAS_GUROBI
        if (solver == ES_MIPSolver::Gurobi)
        {
            env->dualSolver = MIPSolverPtr(new MIPSolverGurobi(env));
            env->process->usedMIPSolver = ES_MIPSolver::Gurobi;
            env->output->outputInfo("Gurobi selected as MIP solver.");
        }
#endif
        if (solver == ES_MIPSolver::Cbc)
        {
            env->dualSolver = MIPSolverPtr(new MIPSolverOsiCbc(env));
            env->process->usedMIPSolver = ES_MIPSolver::Cbc;
            env->output->outputInfo("Cbc selected as MIP solver.");
        }
    }

    env->process->stopTimer("DualStrategy");
}

TaskInitializeDualSolver::~TaskInitializeDualSolver()
{
}

void TaskInitializeDualSolver::run()
{
}
std::string TaskInitializeDualSolver::getType()
{
    std::string type = typeid(this).name();
    return (type);
}