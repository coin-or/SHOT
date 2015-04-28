#pragma once
#include "NLPSolverIPOptBase.h"
#include "../OptProblems/OptProblemNLPRelaxed.h"
#include "../UtilityFunctions.h"

class NLPSolverIPOptRelaxed :
	public INLPSolver, public NLPSolverIPOptBase
{
public:
	NLPSolverIPOptRelaxed();
	~NLPSolverIPOptRelaxed();

	virtual bool createProblem(OSInstance * origInstance);
	virtual bool solveProblem();
	virtual void saveProblemModelToFile(std::string fileName);
	//virtual std::vector<double> getSolution();
	//std::vector<double> NLPpoint;

private:
	//OSOption* osOption;
	OptProblemNLPRelaxed *NLPProblem;
	//DefaultSolver *NLPSolver;
	//std::vector<double> solution;
	//SHOTSettings::Settings *settings;
	//ProcessInfo *processInfo;
	//bool isPointValueCached;

};
