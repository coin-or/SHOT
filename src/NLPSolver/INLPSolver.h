#pragma once
#include "OSInstance.h"
#include <vector>
#include "../OptProblems/OptProblemOriginal.h"

class INLPSolver
{
	public:
		INLPSolver();
		~INLPSolver();

		virtual bool createProblem(OSInstance * origInstance) = 0;
		virtual bool solveProblem() = 0;
		virtual void saveProblemModelToFile(std::string fileName) = 0;
};
