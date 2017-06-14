#pragma once
#include "INLPSolver.h"
#include "NLPSolverBase.h"
#include "NLPIpoptSolver.h"

class NLPSolverIPOptBase: virtual public INLPSolver
{
	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

	protected:
		OSOption* osOption;
		IpoptSolver *NLPSolver;
		OSoLWriter *osolwriter;
		FileUtil *fileUtil;

		std::vector<int> fixedVariableIndexes;
		std::vector<double> fixedVariableValues;

		std::vector<int> startingPointVariableIndexes;
		std::vector<double> startingPointVariableValues;

		virtual E_NLPSolutionStatus solveProblemInstance();

		void fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues);
		void unfixVariables();

		virtual void setInitialSettings();
		virtual void setSolverSpecificInitialSettings() = 0;
		virtual void updateSettings();

		virtual std::vector<double> getCurrentVariableLowerBounds();
		virtual std::vector<double> getCurrentVariableUpperBounds();

		std::vector<double> lowerBoundsBeforeFix;
		std::vector<double> upperBoundsBeforeFix;

	public:

		NLPSolverIPOptBase();
		~NLPSolverIPOptBase();

		virtual void setStartingPoint(std::vector<int> variableIndexes, std::vector<double> variableValues);
		virtual void clearStartingPoint();

		virtual std::vector<double> getSolution();
		virtual double getSolution(int i);
		virtual double getObjectiveValue();

		virtual bool isObjectiveFunctionNonlinear();
		virtual int getObjectiveFunctionVariableIndex();

		virtual void saveOptionsToFile(std::string fileName);

};
