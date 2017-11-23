#pragma once
#include "OptProblemOriginal.h"

class OptProblemOriginalQuadraticObjective: public OptProblemOriginal
{
	public:
		OptProblemOriginalQuadraticObjective();
		~OptProblemOriginalQuadraticObjective();

		virtual bool setProblem(OSInstance *instance);
		//virtual IndexValuePair getMostDeviatingConstraint(std::vector<double> point);
		//virtual void setQuadraticConstraintIndexes();
		//virtual void setNonlinearConstraintIndexes();

	private:

};
