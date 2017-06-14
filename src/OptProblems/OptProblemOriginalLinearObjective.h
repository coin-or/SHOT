#pragma once
#include "OptProblemOriginal.h"

class OptProblemOriginalLinearObjective: public OptProblemOriginal
{
	public:
		OptProblemOriginalLinearObjective();
		~OptProblemOriginalLinearObjective();

		virtual bool setProblem(OSInstance *instance);

	private:

};
