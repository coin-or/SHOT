#pragma once
#include "INLPSolver.h"
#include "NLPSolverBase.h"
#include "NLPIpoptSolver.h"

class NLPSolverIPOptBase :
	public NLPSolverBase
{

public:
	NLPSolverIPOptBase();
	~NLPSolverIPOptBase();

protected:
	OSOption* osOption;
	IpoptSolver *NLPSolver;

	bool isPointValueCached;
	//NLPIpoptSolver *NLPSolver;

	//SmartPtr<NLPIpoptSolver> ipoptSolver;
	//SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
private:
};