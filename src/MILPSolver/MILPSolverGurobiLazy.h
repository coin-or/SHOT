#pragma once
#include "../Enums.h"
#include "IMILPSolver.h"
#include "MILPSolverBase.h"
#include "MILPSolverGurobi.h"

#include "gurobi_c++.h"

#include "../Tasks/TaskSelectPrimalCandidatesFromNLP.h"
#include "../Tasks/TaskInitializeLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsIndividualLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsSolution.h"
#include "../Tasks/TaskUpdateNonlinearObjectiveByLinesearch.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromLinesearch.h"

class MILPSolverGurobiLazy: public MILPSolverGurobi
{
	public:
		MILPSolverGurobiLazy();
		virtual ~MILPSolverGurobiLazy();

		virtual void checkParameters();

		virtual void initializeSolverSettings();

		/*virtual void createIntegerCut(std::vector<int> binaryIndexes)
		 {
		 MILPSolverBase::createIntegerCut(binaryIndexes);
		 }*/

		virtual int increaseSolutionLimit(int increment);
		virtual void setSolutionLimit(long limit);
		virtual int getSolutionLimit();

		E_ProblemSolutionStatus solveProblem();

		int lastSummaryIter = 0;
		int currIter = 0;
		double lastSummaryTimeStamp = 0.0;
		int lastHeaderIter = 0;

	private:
};

class GurobiCallback: public GRBCallback
{
	public:
		GRBVar* vars;

		GurobiCallback(GRBVar* xvars);

	protected:
		void callback();

	private:
		bool isMinimization = true;
		int cbCalls = 0;
		int lastNumAddedHyperplanes = 0;

		int numVar = 0;
		double lastUpdatedPrimal;

		TaskBase *tSelectPrimNLP;
		TaskBase *taskSelectHPPts;
		TaskUpdateNonlinearObjectiveByLinesearch *taskUpdateObjectiveByLinesearch;
		TaskSelectPrimalCandidatesFromLinesearch *taskSelectPrimalSolutionFromLinesearch;

		bool checkFixedNLPStrategy(SolutionPoint point);
		void printIterationReport(SolutionPoint point);

		bool checkAbsoluteObjectiveGapToleranceMet();
		bool checkRelativeObjectiveGapToleranceMet();
		//bool checkRelativeMIPGapToleranceMet();

		void createHyperplane(Hyperplane hyperplane);
		void createIntegerCut(std::vector<int> binaryIndexes);

		void addLazyConstraint(std::vector<SolutionPoint> candidatePoints);
};
