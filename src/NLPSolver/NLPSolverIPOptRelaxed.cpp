#include "NLPSolverIPOptRelaxed.h"

NLPSolverIPOptRelaxed::NLPSolverIPOptRelaxed()
{
	//processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

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
	auto constrTol = settings->getDoubleSetting("ConstraintToleranceInteriorPointStrategy", "Ipopt");
	osOption->setAnotherSolverOption("constr_viol_tol", UtilityFunctions::toStringFormat(constrTol, "%.10f"), "ipopt",
			"", "double", "");

	osOption->setAnotherSolverOption("tol",
			UtilityFunctions::toStringFormat(settings->getDoubleSetting("ToleranceInteriorPointStrategy", "Ipopt"),
					"%.10f"), "ipopt", "", "double", "");

	osOption->setAnotherSolverOption("max_iter",
			to_string(settings->getIntSetting("MaxIterInteriorPointStrategy", "Ipopt")), "ipopt", "", "integer", "");

	auto timeLimit = settings->getDoubleSetting("NLPTimeLimit", "PrimalBound");
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
