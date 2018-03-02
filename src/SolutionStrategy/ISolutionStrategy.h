#pragma once
#include "OSInstance.h"
#include "ProcessInfo.h"

//class SHOTSolver;

class ISolutionStrategy
{
  public:
	~ISolutionStrategy()
	{
		delete ProcessInfo::getInstance().MIPSolver;
	}

	virtual void initializeStrategy() = 0;
	virtual bool solveProblem() = 0;
};
