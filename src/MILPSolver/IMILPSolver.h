#pragma once
#include <vector>
#include "../Enums.h"
#include "OSInstance.h"
#include "../ProcessInfo.h"
#include "../OptProblems/OptProblemOriginal.h"
#include "../OptProblems/OptProblemOriginalLinearObjective.h"
#include "../OptProblems/OptProblemOriginalQuadraticObjective.h"
#include "../OptProblems/OptProblemOriginalNonlinearObjective.h"

class IMILPSolver
{
	public:
		//MILPSolver() {};

		virtual ~IMILPSolver()
		{
		}
		;

		virtual bool createLinearProblem(OptProblem *origProblem) = 0;
		virtual std::vector<double> getVariableSolution() = 0;
		virtual void activateDiscreteVariables(bool activate) = 0;
		virtual bool getDiscreteVariableStatus() = 0;
		virtual E_ProblemSolutionStatus solveProblem() = 0;
		virtual E_ProblemSolutionStatus getSolutionStatus() = 0;
		virtual double getLastObjectiveValue() = 0;
		virtual double getBestObjectiveValue() = 0;

		virtual int increaseSolutionLimit(int increment) = 0;
		virtual void setSolutionLimit(int limit) = 0;
		virtual int getSolutionLimit() = 0;

		virtual void writeProblemToFile(std::string filename) = 0;

		virtual std::vector<SolutionPoint> getAllVariableSolutions() = 0;
		virtual bool addLinearConstraint(std::vector<IndexValuePair> elements, int numNonZero, double constant) = 0;

		virtual void setTimeLimit(double seconds) = 0;

		virtual void setCutOff(double cutOff) = 0;

		virtual void addMIPStart(std::vector<double> point) = 0;
		virtual void deleteMIPStarts() = 0;

		virtual void changeConstraintToLazy(std::vector<int> constrIdxs) = 0;

	protected:
};
