/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverIpoptMinimax.h"

NLPSolverIpoptMinimax::NLPSolverIpoptMinimax(EnvironmentPtr envPtr) : INLPSolver(envPtr)
{
    osolwriter = new OSoLWriter();

    NLPProblem = new OptProblemNLPMinimax(env);

    setInitialSettings();
}

NLPSolverIpoptMinimax::~NLPSolverIpoptMinimax()
{
    delete osolwriter;
    delete NLPProblem;
}

DoubleVector NLPSolverIpoptMinimax::getSolution()
{
    int numVar = NLPProblem->getNumberOfVariables();
    DoubleVector tmpPoint(numVar);

    for (int i = 0; i < numVar; i++)
    {
        tmpPoint.at(i) = NLPSolverIpoptBase::getSolution(i);
    }

    if (env->model->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
    {
        tmpPoint.pop_back();
    }

    // Removes the minimax objective variable from solution
    tmpPoint.pop_back();

    return (tmpPoint);
}

bool NLPSolverIpoptMinimax::createProblemInstance(OSInstance *origInstance)
{
    env->output->outputInfo("     Creating Ipopt minimax problem.");
    dynamic_cast<OptProblemNLPMinimax *>(NLPProblem)->reformulate(origInstance);
    env->output->outputInfo("     Ipopt minimax problem created.");

    return (true);
}

void NLPSolverIpoptMinimax::setSolverSpecificInitialSettings()
{
    auto constrTol = env->settings->getDoubleSetting("Ipopt.ConstraintViolationTolerance", "Subsolver");
    osOption->setAnotherSolverOption("constr_viol_tol", UtilityFunctions::toStringFormat(constrTol, "%.10f"), "ipopt",
                                     "", "double", "");

    osOption->setAnotherSolverOption("tol",
                                     UtilityFunctions::toStringFormat(
                                         env->settings->getDoubleSetting("Ipopt.RelativeConvergenceTolerance", "Subsolver"), "%.10f"),
                                     "ipopt", "", "double", "");

    osOption->setAnotherSolverOption("max_iter",
                                     std::to_string(env->settings->getIntSetting("Ipopt.MaxIterations", "Subsolver")), "ipopt", "",
                                     "integer", "");

    auto timeLimit = env->settings->getDoubleSetting("FixedInteger.TimeLimit", "Primal");
    osOption->setAnotherSolverOption("max_cpu_time", UtilityFunctions::toStringFormat(timeLimit, "%.10f"), "ipopt", "",
                                     "number", "");
}
