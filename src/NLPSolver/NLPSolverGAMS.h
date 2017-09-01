/**
 * @file NLPSolverGAMS.h
 */

#pragma once
#include "NLPSolverBase.h"


class NLPSolverGAMS : public NLPSolverBase
{
public:
	NLPSolverGAMS() {};
	~NLPSolverGAMS() {};

//	virtual void setProblem(OSInstance * origInstance) = 0;
//	virtual void initializeProblem() = 0;
	virtual void setStartingPoint(std::vector<int> variableIndexes, std::vector<double> variableValues) {}
	virtual void clearStartingPoint() {}

//	virtual E_NLPSolutionStatus solveProblem() = 0;
	virtual void fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues) {}

	virtual void unfixVariables() {}

//	virtual void saveProblemToFile(std::string fileName) = 0;
	virtual void saveOptionsToFile(std::string fileName) {}

	virtual std::vector<double> getSolution() {};
	virtual double getSolution(int i) {}
	virtual double getObjectiveValue() {}

	virtual bool isObjectiveFunctionNonlinear() {}
	virtual int getObjectiveFunctionVariableIndex() {}

//	virtual std::vector<double> getVariableLowerBounds() = 0;
//	virtual std::vector<double> getVariableUpperBounds() = 0;

protected:
	virtual E_NLPSolutionStatus solveProblemInstance() {}
	virtual bool createProblemInstance(OSInstance * origInstance) {}

	virtual std::vector<double> getCurrentVariableLowerBounds() {}
	virtual std::vector<double> getCurrentVariableUpperBounds() {}


protected:

//	bool createProblemInstance(OSInstance * origInstance);

//	virtual void setSolverSpecificInitialSettings();

private:

//	SHOTSettings::Settings *settings;
//	ProcessInfo *processInfo;

};
