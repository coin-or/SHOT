#pragma once
#include "OSInstance.h"

//class SHOTSolver;

class ISolutionStrategy
{
	public:
		~ISolutionStrategy()
		{
		}
		;

		virtual void initializeStrategy() = 0;
		virtual bool solveProblem() = 0;

};
