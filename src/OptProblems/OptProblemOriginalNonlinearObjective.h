#pragma once
#include "OptProblemOriginal.h"

class OptProblemOriginalNonlinearObjective: public OptProblemOriginal
{
	public:
		OptProblemOriginalNonlinearObjective();
		~OptProblemOriginalNonlinearObjective();

		virtual bool setProblem(OSInstance *instance);
		double calculateConstraintFunctionValue(int idx, std::vector<double> point);
		SparseVector* calculateConstraintFunctionGradient(int idx, std::vector<double> point);
		virtual int getNumberOfNonlinearConstraints();
		virtual int getNumberOfConstraints();
		virtual std::vector<std::string> getConstraintNames();
		virtual int getNumberOfVariables();
		virtual int getNumberOfRealVariables();
		virtual std::vector<std::string> getVariableNames();
		virtual std::vector<char> getVariableTypes();

		virtual std::vector<double> getVariableLowerBounds();
		virtual std::vector<double> getVariableUpperBounds();

		virtual void setVariableUpperBound(int varIdx, double value);
		virtual void setVariableLowerBound(int varIdx, double value);

		virtual std::vector<std::pair<int, double>> getObjectiveFunctionVarCoeffPairs();
		virtual double getObjectiveConstant();

		IndexValuePair getMostDeviatingAllConstraint(std::vector<double> point);

		virtual void setNonlinearConstraintIndexes();

	private:

		std::string addedObjectiveVariableName;
		std::string addedConstraintName;
		double addedObjectiveVariableLowerBound;
		double addedObjectiveVariableUpperBound;

};

