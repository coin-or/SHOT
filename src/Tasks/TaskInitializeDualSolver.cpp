/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskInitializeDualSolver.h"

TaskInitializeDualSolver::TaskInitializeDualSolver(ES_MIPSolver solver, bool useLazyStrategy)
{
    ProcessInfo::getInstance().startTimer("DualStrategy");

    if (solver != ES_MIPSolver::Cplex && solver != ES_MIPSolver::Gurobi && solver != ES_MIPSolver::Cbc)
    {
        Output::getInstance().Output::getInstance().outputError("Error in solver definition. Check option 'Dual.MIP.Solver'.");
        throw new ErrorClass("Error in MIP solver definition.  Check option 'Dual.MIP.Solver'.");
    }

    if (useLazyStrategy)
    {
#ifdef HAS_CPLEX
        if (solver == ES_MIPSolver::Cplex)
        {
#ifdef HAS_CPLEX_NEW_CALLBACK
            if (Settings::getInstance().getBoolSetting("Cplex.UseNewCallbackType", "Subsolver"))
            {
                ProcessInfo::getInstance().MIPSolver = new MIPSolverCplexLazy();
                ProcessInfo::getInstance().usedMIPSolver = ES_MIPSolver::Cplex;
                Output::getInstance().outputInfo("Cplex with lazy callbacks selected as MIP solver.");
            }
            else
#endif
            {
                ProcessInfo::getInstance().MIPSolver = new MIPSolverCplexLazyOriginalCallback();
                ProcessInfo::getInstance().usedMIPSolver = ES_MIPSolver::Cplex;
                Output::getInstance().outputInfo("Cplex with original lazy callbacks selected as MIP solver.");
            }
        }
#endif

#ifdef HAS_GUROBI
        if (solver == ES_MIPSolver::Gurobi)
        {
            ProcessInfo::getInstance().MIPSolver = new MIPSolverGurobiLazy();
            ProcessInfo::getInstance().usedMIPSolver = ES_MIPSolver::Gurobi;
            Output::getInstance().outputInfo("Gurobi with lazy callbacks selected as MIP solver.");
        }
#endif

        if (solver == ES_MIPSolver::Cbc)
        {
            ProcessInfo::getInstance().MIPSolver = new MIPSolverOsiCbc();
            ProcessInfo::getInstance().usedMIPSolver = ES_MIPSolver::Cbc;
            Output::getInstance().outputInfo("Cbc selected as MIP solver.");
        }
    }
    else
    {

#ifdef HAS_CPLEX
        if (solver == ES_MIPSolver::Cplex)
        {
            ProcessInfo::getInstance().MIPSolver = new MIPSolverCplex();
            Output::getInstance().outputInfo("Cplex selected as MIP solver.");
        }
#endif

#ifdef HAS_GUROBI
        if (solver == ES_MIPSolver::Gurobi)
        {
            ProcessInfo::getInstance().MIPSolver = new MIPSolverGurobi();
            Output::getInstance().outputInfo("Gurobi selected as MIP solver.");
        }
#endif
        if (solver == ES_MIPSolver::Cbc)
        {
            ProcessInfo::getInstance().MIPSolver = new MIPSolverOsiCbc();
            Output::getInstance().outputInfo("Cbc selected as MIP solver.");
        }
    }

    ProcessInfo::getInstance().stopTimer("DualStrategy");
}

TaskInitializeDualSolver::~TaskInitializeDualSolver()
{
    delete ProcessInfo::getInstance().MIPSolver;
}

void TaskInitializeDualSolver::run()
{
}
std::string TaskInitializeDualSolver::getType()
{
    std::string type = typeid(this).name();
    return (type);
}