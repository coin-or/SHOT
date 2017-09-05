#include "NLPSolverIPOptMinimax.h"

NLPSolverIPOptMinimax::NLPSolverIPOptMinimax()
{
	//processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	osolwriter = new OSoLWriter();

	NLPProblem = new OptProblemNLPMinimax();

	setInitialSettings();
}

NLPSolverIPOptMinimax::~NLPSolverIPOptMinimax()
{
	delete NLPSolver;
	delete osolwriter;
	delete NLPProblem;
}

std::vector<double> NLPSolverIPOptMinimax::getSolution()
{
	int numVar = NLPProblem->getNumberOfVariables();
	std::vector<double> tmpPoint(numVar);

	for (int i = 0; i < numVar; i++)
	{
		tmpPoint.at(i) = NLPSolverIPOptBase::getSolution(i);
	}

	if (ProcessInfo::getInstance().originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
	{
		tmpPoint.pop_back();
	}

	// Removes the minimax objective variable from solution
	tmpPoint.pop_back();

	return (tmpPoint);
}

bool NLPSolverIPOptMinimax::createProblemInstance(OSInstance * origInstance)
{

	ProcessInfo::getInstance().outputInfo("     Creating Ipopt minimax problem.");

	dynamic_cast<OptProblemNLPMinimax*>(NLPProblem)->reformulate(origInstance);

	ProcessInfo::getInstance().outputInfo("     Ipopt minimax problem created.");

	return (true);
}

void NLPSolverIPOptMinimax::setSolverSpecificInitialSettings()
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
