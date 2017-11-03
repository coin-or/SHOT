#pragma once
#include "OptProblem.h"

class OptProblemNLPMinimax: public OptProblem
{
	public:
		OptProblemNLPMinimax();
		~OptProblemNLPMinimax();

		void reformulate(OSInstance *originalProblem);
		//virtual double calculateConstraintFunctionValue(int idx, std::vector<double> point);

		virtual void copyVariables(OSInstance *source, OSInstance *destination, bool integerRelaxed);
		virtual void copyObjectiveFunction(OSInstance *source, OSInstance *destination);
		virtual void copyConstraints(OSInstance *source, OSInstance *destination);
		virtual void copyLinearTerms(OSInstance *source, OSInstance *destination);
		virtual void copyQuadraticTerms(OSInstance *source, OSInstance *destination);
		virtual void copyNonlinearExpressions(OSInstance *source, OSInstance *destination);

		//IndexValuePair getMostDeviatingConstraint(std::vector<double> point);
		//bool isConstraintsFulfilledInPoint(std::vector<double> point, double eps);

		//double calculateConstraintFunctionValue(int idx, std::vector<double> point);
		//SparseVector* calculateConstraintFunctionGradient(int idx, std::vector<double> point);

	private:

		int muindex;

};
