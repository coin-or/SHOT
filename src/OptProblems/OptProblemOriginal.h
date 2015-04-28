#pragma once
#include "OptProblem.h"
#include "../UtilityFunctions.h"

class OptProblemOriginal :
	public OptProblem
{
public:
	OptProblemOriginal();
	~OptProblemOriginal();

	//bool readProblemFromFile(std::string fileName);
	virtual bool setProblem(OSInstance *instance) = 0;
	//bool checkProblemScope();


	//virtual std::vector<std::pair<int, double>> getObjectiveFunctionVarCoeffPairs();
	//virtual double getObjectiveConstant();

	//virtual bool isConstraintsFulfilledInPoint(std::vector<double> point);
	//virtual bool isConstraintsFulfilledInPoint(std::vector<double> point, double eps);
	virtual double calculateConstraintFunctionValue(int idx, std::vector<double> point);
	virtual SparseVector* calculateConstraintFunctionGradient(int idx, std::vector<double> point);

	//virtual int getNumberOfNonlinearConstraints();
	//virtual int getNumberOfConstraints();
	//virtual std::vector<std::string> getConstraintNames();
	//virtual IndexValuePair getMostDeviatingConstraint(std::vector<double> point);
	//std::vector<IndexValuePair> getMostDeviatingConstraints(std::vector<double> point, double fraction);

	//virtual int getNumberOfVariables();
	//virtual int getNumberOfRealVariables();
	//virtual std::vector<std::string> getVariableNames();
	//virtual std::vector<char> getVariableTypes();
	//virtual std::vector<double> getVariableLowerBounds();
	//virtual std::vector<double> getVariableUpperBounds();

};
