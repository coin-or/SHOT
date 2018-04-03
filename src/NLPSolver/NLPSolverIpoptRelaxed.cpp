/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverIpoptRelaxed.h"

NLPSolverIpoptRelaxed::NLPSolverIpoptRelaxed()
{
    osolwriter = new OSoLWriter();

    NLPProblem = new OptProblemNLPRelaxed();

    setInitialSettings();
}

NLPSolverIpoptRelaxed::~NLPSolverIpoptRelaxed()
{
    delete osolwriter;
    delete NLPProblem;
}

bool NLPSolverIpoptRelaxed::createProblemInstance(OSInstance *origInstance)
{
    Output::getInstance().outputInfo("     Creating relaxed Ipopt problem.");

    dynamic_cast<OptProblemNLPRelaxed *>(NLPProblem)->reformulate(origInstance);

    Output::getInstance().outputInfo("     Ipopt relaxed NLP problem created.");

    return (true);
}

void NLPSolverIpoptRelaxed::setSolverSpecificInitialSettings()
{
    auto constrTol = Settings::getInstance().getDoubleSetting("Ipopt.ConstraintViolationTolerance", "Subsolver");
    osOption->setAnotherSolverOption("constr_viol_tol", UtilityFunctions::toStringFormat(constrTol, "%.10f"), "ipopt",
                                     "", "double", "");

    osOption->setAnotherSolverOption("tol",
                                     UtilityFunctions::toStringFormat(
                                         Settings::getInstance().getDoubleSetting("Ipopt.RelativeConvergenceTolerance", "Subsolver"), "%.10f"),
                                     "ipopt", "", "double", "");

    osOption->setAnotherSolverOption("max_iter",
                                     to_string(Settings::getInstance().getIntSetting("Ipopt.MaxIterations", "Subsolver")), "ipopt", "",
                                     "integer", "");

    auto timeLimit = Settings::getInstance().getDoubleSetting("FixedInteger.TimeLimit", "Primal");
    osOption->setAnotherSolverOption("max_cpu_time", UtilityFunctions::toStringFormat(timeLimit, "%.10f"), "ipopt", "",
                                     "number", "");
}

std::vector<double> NLPSolverIpoptRelaxed::getSolution()
{
    int numVar = NLPProblem->getNumberOfVariables();
    std::vector<double> tmpPoint(numVar);

    for (int i = 0; i < numVar; i++)
    {
        tmpPoint.at(i) = NLPSolverIpoptBase::getSolution(i);
    }

    return (tmpPoint);
}
