#include "NLPSolverIPOptRelaxed.h"

NLPSolverIPOptRelaxed::NLPSolverIPOptRelaxed()
{

	osolwriter = new OSoLWriter();

	NLPProblem = new OptProblemNLPRelaxed();

	setInitialSettings();
}

NLPSolverIPOptRelaxed::~NLPSolverIPOptRelaxed()
{
	delete NLPSolver;
	delete osolwriter;
	delete NLPProblem;
}

bool NLPSolverIPOptRelaxed::createProblemInstance(OSInstance * origInstance)
{
	ProcessInfo::getInstance().outputInfo("     Creating relaxed Ipopt problem.");

	dynamic_cast<OptProblemNLPRelaxed*>(NLPProblem)->reformulate(origInstance);

	ProcessInfo::getInstance().outputInfo("     Ipopt relaxed NLP problem created.");

	return (true);
}

void NLPSolverIPOptRelaxed::setSolverSpecificInitialSettings()
{
	auto constrTol = Settings::getInstance().getDoubleSetting("ConstraintToleranceInteriorPointStrategy", "Ipopt");
	osOption->setAnotherSolverOption("constr_viol_tol", UtilityFunctions::toStringFormat(constrTol, "%.10f"), "ipopt",
			"", "double", "");

	osOption->setAnotherSolverOption("tol",
			UtilityFunctions::toStringFormat(
					Settings::getInstance().getDoubleSetting("ToleranceInteriorPointStrategy", "Ipopt"), "%.10f"),
			"ipopt", "", "double", "");

	osOption->setAnotherSolverOption("max_iter",
			to_string(Settings::getInstance().getIntSetting("MaxIterInteriorPointStrategy", "Ipopt")), "ipopt", "",
			"integer", "");

	auto timeLimit = Settings::getInstance().getDoubleSetting("NLPTimeLimit", "PrimalBound");
	osOption->setAnotherSolverOption("max_cpu_time", UtilityFunctions::toStringFormat(timeLimit, "%.10f"), "ipopt", "",
			"number", "");

}

std::vector<double> NLPSolverIPOptRelaxed::getSolution()
{
	int numVar = NLPProblem->getNumberOfVariables();
	std::vector<double> tmpPoint(numVar);

	for (int i = 0; i < numVar; i++)
	{
		tmpPoint.at(i) = NLPSolverIPOptBase::getSolution(i);
	}

	return (tmpPoint);
}
