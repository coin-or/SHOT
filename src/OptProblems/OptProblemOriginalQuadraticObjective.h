#pragma once
#include "OptProblemOriginal.h"

class OptProblemOriginalQuadraticObjective: public OptProblemOriginal
{
	public:
		OptProblemOriginalQuadraticObjective();
		~OptProblemOriginalQuadraticObjective();

		virtual bool setProblem(OSInstance *instance);

	private:

};
