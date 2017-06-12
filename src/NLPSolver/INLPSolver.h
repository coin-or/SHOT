#pragma once
#include "OSInstance.h"
#include "vector"
#include "../OptProblems/OptProblemOriginal.h"

class INLPSolver
{
	public:
		INLPSolver();
		~INLPSolver();

		OptProblem *NLPProblem;

		virtual void setProblem(OSInstance * origInstance) = 0;
		virtual void initializeProblem() = 0;
		virtual void setStartingPoint(std::vector<int> variableIndexes, std::vector<double> variableValues) = 0;
		virtual void clearStartingPoint() = 0;

		virtual E_NLPSolutionStatus solveProblem() = 0;
		virtual void fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues) = 0;

		virtual void unfixVariables() = 0;

		virtual void saveProblemToFile(std::string fileName) = 0;
		virtual void saveOptionsToFile(std::string fileName) = 0;

		virtual std::vector<double> getSolution() = 0;
		virtual double getSolution(int i) = 0;
		virtual double getObjectiveValue() = 0;

		virtual bool isObjectiveFunctionNonlinear() = 0;
		virtual int getObjectiveFunctionVariableIndex() = 0;

		virtual std::vector<double> getVariableLowerBounds() = 0;
		virtual std::vector<double> getVariableUpperBounds() = 0;

	protected:
		virtual E_NLPSolutionStatus solveProblemInstance() = 0;
		virtual bool createProblemInstance(OSInstance * origInstance) = 0;

		virtual std::vector<double> getCurrentVariableLowerBounds() = 0;
		virtual std::vector<double> getCurrentVariableUpperBounds() = 0;

};
