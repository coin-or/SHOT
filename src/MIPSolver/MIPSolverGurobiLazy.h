#pragma once
#include "../Enums.h"
#include "IMIPSolver.h"
#include "MIPSolverBase.h"
#include "MIPSolverGurobi.h"
#include "MIPSolverCallbackBase.h"

#include "gurobi_c++.h"

class MIPSolverGurobiLazy: public MIPSolverGurobi
{
	public:
		MIPSolverGurobiLazy();
		virtual ~MIPSolverGurobiLazy();

		virtual void checkParameters();

		virtual void initializeSolverSettings();

		virtual int increaseSolutionLimit(int increment);
		virtual void setSolutionLimit(long limit);
		virtual int getSolutionLimit();

		E_ProblemSolutionStatus solveProblem();

	private:
};

class GurobiCallback: public GRBCallback, public MIPSolverCallbackBase
{
	public:
		GRBVar* vars;

		GurobiCallback(GRBVar* xvars);

	protected:
		void callback();

	private:
		int numVar = 0;
	
		//void printIterationReport(SolutionPoint point);

		void createHyperplane(Hyperplane hyperplane);
		void createIntegerCut(std::vector<int> binaryIndexes);

		void addLazyConstraint(std::vector<SolutionPoint> candidatePoints);
};