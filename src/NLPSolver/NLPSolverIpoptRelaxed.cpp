/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverIpoptRelaxed.h"

namespace SHOT
{

NLPSolverIpoptRelaxed::NLPSolverIpoptRelaxed(EnvironmentPtr envPtr) : INLPSolver(envPtr)
{
    osolwriter = new OSoLWriter();

    NLPProblem = new OptProblemNLPRelaxed(env);

    setInitialSettings();
}

NLPSolverIpoptRelaxed::~NLPSolverIpoptRelaxed()
{
    delete osolwriter;
    delete NLPProblem;
}

bool NLPSolverIpoptRelaxed::createProblemInstance(OSInstance *origInstance)
{
    env->output->outputInfo("     Creating relaxed Ipopt problem.");

    dynamic_cast<OptProblemNLPRelaxed *>(NLPProblem)->reformulate(origInstance);

    env->output->outputInfo("     Ipopt relaxed NLP problem created.");

    return (true);
}

void NLPSolverIpoptRelaxed::setSolverSpecificInitialSettings()
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

VectorDouble NLPSolverIpoptRelaxed::getSolution()
{
    int numVar = NLPProblem->getNumberOfVariables();
    VectorDouble tmpPoint(numVar);

    for (int i = 0; i < numVar; i++)
    {
        tmpPoint.at(i) = NLPSolverIpoptBase::getSolution(i);
    }

    return (tmpPoint);
}
} // namespace SHOT