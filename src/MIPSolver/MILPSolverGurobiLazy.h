#pragma once
#include "../Enums.h"
#include "IMILPSolver.h"
#include "MILPSolverBase.h"
#include "MILPSolverGurobi.h"
#include "MILPSolverCallbackBase.h"

#include "gurobi_c++.h"

class MILPSolverGurobiLazy: public MILPSolverGurobi
{
	public:
		MILPSolverGurobiLazy();
		virtual ~MILPSolverGurobiLazy();

		virtual void checkParameters();

		virtual void initializeSolverSettings();

		virtual int increaseSolutionLimit(int increment);
		virtual void setSolutionLimit(long limit);
		virtual int getSolutionLimit();

		E_ProblemSolutionStatus solveProblem();

	private:
};

class GurobiCallback: public GRBCallback, public MILPSolverCallbackBase
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