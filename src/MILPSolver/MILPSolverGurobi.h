#pragma once
#include "../Enums.h"
#include "IMILPSolver.h"
#include "gurobi_c++.h"
#include "MILPSolverBase.h"
class MILPSolverGurobi: public IMILPSolver, MILPSolverBase
{
	public:
		MILPSolverGurobi();
		virtual ~MILPSolverGurobi();

		virtual bool createLinearProblem(OptProblem *origProblem);
		virtual bool addLinearConstraint(std::vector<IndexValuePair> elements, int numNonZero, double constant);
		virtual std::vector<double> getVariableSolution();
		virtual void activateDiscreteVariables(bool activate);
		virtual bool getDiscreteVariableStatus();
		virtual E_ProblemSolutionStatus solveProblem();
		virtual E_ProblemSolutionStatus getSolutionStatus();
		virtual double getLastObjectiveValue();
		virtual double getBestObjectiveValue();

		virtual int increaseSolutionLimit(int increment);
		virtual void setSolutionLimit(int limit);
		virtual int getSolutionLimit();

		virtual void writeProblemToFile(std::string filename);

		virtual std::vector<SolutionPoint> getAllVariableSolutions();

		virtual void setTimeLimit(double seconds);

		virtual void setCutOff(double cutOff);

		virtual void addMIPStart(std::vector<double> point);
		virtual void deleteMIPStarts();
		virtual void changeConstraintToLazy(std::vector<int> constrIdxs);
};
