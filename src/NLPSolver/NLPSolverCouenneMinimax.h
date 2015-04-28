#pragma once
#include "INLPSolver.h"
#include "NLPSolverBase.h"
#include "../OptProblems/OptProblemNLPMinimax.h"
#include "OSDefaultSolver.h"

#include "OSCouenneSolver.h"

class NLPSolverCouenneMinimax :
	public NLPSolverBase, public INLPSolver
{
public:
	NLPSolverCouenneMinimax();
	~NLPSolverCouenneMinimax();

	virtual bool createProblem(OSInstance * origInstance);
	virtual bool solveProblem();
	virtual void saveProblemModelToFile(std::string fileName);

private: 

	DefaultSolver *NLPSolver;
	OptProblemNLPMinimax *NLPProblem;

protected:
	OSOption* osOption;

	bool isPointValueCached;
};

