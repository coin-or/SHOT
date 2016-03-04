#pragma once
#include <vector>
#include "../Enums.h"
#include "OSInstance.h"
#include "../ProcessInfo.h"
#include "../OptProblems/OptProblemOriginal.h"
#include "../OptProblems/OptProblemOriginalLinearObjective.h"
#include "../OptProblems/OptProblemOriginalQuadraticObjective.h"
#include "../OptProblems/OptProblemOriginalNonlinearObjective.h"

struct GeneratedHyperplane
{
		int generatedConstraintIndex;
		int sourceConstraintIndex;
		std::vector<double> generatedPoint;
		E_HyperplaneSource source;
		bool isLazy = false;
		bool isRemoved = false;
		int generatedIter;
		int removedIter;
		int convertedToLazyIter;
};

class IMILPSolver
{
	public:
		//MILPSolver() {};

		virtual ~IMILPSolver()
		{
		}
		;

		virtual void checkParameters() = 0;

		virtual bool createLinearProblem(OptProblem *origProblem) = 0;
		virtual void initializeSolverSettings() = 0;

		virtual std::vector<double> getVariableSolution(int solIdx) = 0;
		virtual int getNumberOfSolutions() = 0;

		virtual void activateDiscreteVariables(bool activate) = 0;
		virtual bool getDiscreteVariableStatus() = 0;
		virtual E_ProblemSolutionStatus solveProblem() = 0;
		virtual E_ProblemSolutionStatus getSolutionStatus() = 0;
		virtual double getObjectiveValue() = 0;

		virtual double getDualObjectiveValue() = 0;

		virtual double getObjectiveValue(int solIdx) = 0;

		virtual int increaseSolutionLimit(int increment) = 0;
		virtual void setSolutionLimit(int limit) = 0;
		virtual int getSolutionLimit() = 0;

		virtual void writeProblemToFile(std::string filename) = 0;

		virtual std::vector<SolutionPoint> getAllVariableSolutions() = 0;
		virtual int addLinearConstraint(std::vector<IndexValuePair> elements, double constant) = 0;
		virtual int addLinearConstraint(std::vector<IndexValuePair> elements, double constant, bool isGreaterThan) = 0;

		virtual void setTimeLimit(double seconds) = 0;

		virtual void setCutOff(double cutOff) = 0;

		virtual void addMIPStart(std::vector<double> point) = 0;
		virtual void deleteMIPStarts() = 0;

		virtual void changeConstraintToLazy(GeneratedHyperplane &hyperplane) = 0;

		virtual void populateSolutionPool() = 0;

		virtual void fixVariable(int varIndex, double value) = 0;
		virtual void updateVariableBound(int varIndex, double lowerBound, double upperBound) = 0;

		virtual pair<double, double> getCurrentVariableBounds(int varIndex) = 0;
		virtual void createHyperplane(Hyperplane hyperplane)= 0;
		virtual void createInteriorHyperplane(Hyperplane hyperplane) = 0;

		virtual bool supportsQuadraticObjective() = 0;
		virtual bool supportsQuadraticConstraints() = 0;
		virtual bool supportsLazyConstraints() = 0;

		virtual std::vector<GeneratedHyperplane>* getGeneratedHyperplanes() = 0;
	protected:
};
