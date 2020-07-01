/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskInitializeDualSolver.h"

#include "../DualSolver.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"

#include "../MIPSolver/IMIPSolver.h"

#ifdef HAS_CPLEX
#include "../MIPSolver/MIPSolverCplex.h"
#include "../MIPSolver/MIPSolverCplexSingleTree.h"
#include "../MIPSolver/MIPSolverCplexSingleTreeLegacy.h"
#endif

#ifdef HAS_GUROBI
#include "../MIPSolver/MIPSolverGurobi.h"
#include "../MIPSolver/MIPSolverGurobiSingleTree.h"
#endif

#ifdef HAS_CBC
#include "../MIPSolver/MIPSolverCbc.h"
#endif

namespace SHOT
{

TaskInitializeDualSolver::TaskInitializeDualSolver(EnvironmentPtr envPtr, bool useLazyStrategy) : TaskBase(envPtr)
{
    auto solver = static_cast<ES_MIPSolver>(env->settings->getSetting<int>("MIP.Solver", "Dual"));

    env->timing->startTimer("DualStrategy");

    bool solverSelected = false;

    if(useLazyStrategy)
    {
#ifdef HAS_CPLEX
        if(solver == ES_MIPSolver::Cplex)
        {
            if(env->settings->getSetting<bool>("Cplex.UseGenericCallback", "Subsolver"))
            {
                env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverCplexSingleTree>(env));
                env->results->usedMIPSolver = ES_MIPSolver::Cplex;
                env->output->outputDebug(" Cplex with lazy callbacks selected as MIP solver.");
                solverSelected = true;
            }
            else
            {
                env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverCplexSingleTreeLegacy>(env));
                env->results->usedMIPSolver = ES_MIPSolver::Cplex;
                env->output->outputDebug(" Cplex with original lazy callbacks selected as MIP solver.");
                solverSelected = true;
            }
        }
#endif

#ifdef HAS_GUROBI
        if(solver == ES_MIPSolver::Gurobi)
        {
            env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverGurobiSingleTree>(env));
            env->results->usedMIPSolver = ES_MIPSolver::Gurobi;
            env->output->outputDebug(" Gurobi with lazy callbacks selected as MIP solver.");
            solverSelected = true;
        }
#endif

#ifdef HAS_CBC
        if(solver == ES_MIPSolver::Cbc)
        {
            env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverCbc>(env));
            env->results->usedMIPSolver = ES_MIPSolver::Cbc;
            env->output->outputDebug(" Cbc selected as MIP solver.");
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
            env->output->outputDebug(" Cplex selected as MIP solver.");
            solverSelected = true;
        }
#endif

#ifdef HAS_GUROBI
        if(solver == ES_MIPSolver::Gurobi)
        {
            env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverGurobi>(env));
            env->results->usedMIPSolver = ES_MIPSolver::Gurobi;
            env->output->outputDebug(" Gurobi selected as MIP solver.");
            solverSelected = true;
        }
#endif

#ifdef HAS_CBC
        if(solver == ES_MIPSolver::Cbc)
        {
            env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverCbc>(env));
            env->results->usedMIPSolver = ES_MIPSolver::Cbc;
            env->output->outputDebug(" Cbc selected as MIP solver.");
            solverSelected = true;
        }
#endif
    }

    if(!solverSelected)
    {
        env->output->outputWarning(" SHOT has not been compiled with support for selected MIP solver.");

#ifdef HAS_CBC
        env->dualSolver->MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverCbc>(env));
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

    if(!env->dualSolver->MIPSolver->initializeProblem())
        throw Exception(" Cannot initialize selected MIP solver.");

    env->timing->stopTimer("DualStrategy");
}

TaskInitializeDualSolver::~TaskInitializeDualSolver() = default;

void TaskInitializeDualSolver::run() { }
std::string TaskInitializeDualSolver::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT