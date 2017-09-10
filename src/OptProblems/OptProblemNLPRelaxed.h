#pragma once
#include "OptProblem.h"

class OptProblemNLPRelaxed: public OptProblem
{
	public:
		OptProblemNLPRelaxed();
		~OptProblemNLPRelaxed();

		void reformulate(OSInstance *originalProblem);

		//virtual void copyVariables(OSInstance *source, OSInstance *destination, bool integerRelaxed);
		//virtual void copyObjectiveFunction(OSInstance *source, OSInstance *destination);
		//virtual void copyConstraints(OSInstance *source, OSInstance *destination);
		//virtual	void copyLinearTerms(OSInstance *source, OSInstance *destination);
		//virtual void copyQuadraticTerms(OSInstance *source, OSInstance *destination);
		//virtual void copyNonlinearExpressions(OSInstance *source, OSInstance *destination);

		IndexValuePair getMostDeviatingConstraint(std::vector<double> point);
		bool isConstraintsFulfilledInPoint(std::vector<double> point, double eps);
};
