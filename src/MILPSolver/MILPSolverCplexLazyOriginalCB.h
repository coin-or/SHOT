#pragma once
#include "IMILPSolver.h"
#include "MILPSolverBase.h"

#include <functional>
#include <thread>
#include <mutex>

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wignored-attributes"
#endif
#include "ilcplex/ilocplex.h"
#ifdef __GNUC__
#pragma GCC diagnostic warning "-Wignored-attributes"
#endif

#include "../Tasks/TaskSelectPrimalCandidatesFromNLP.h"

#include "../Tasks/TaskInitializeLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsIndividualLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsSolution.h"
#include "../Tasks/TaskUpdateNonlinearObjectiveByLinesearch.h"

class MILPSolverCplexLazyOriginalCB: public MILPSolverCplex
{
	public:

		MILPSolverCplexLazyOriginalCB();
		virtual ~MILPSolverCplexLazyOriginalCB();

		virtual void checkParameters();

		virtual void initializeSolverSettings();

		virtual E_ProblemSolutionStatus solveProblem();

		virtual int increaseSolutionLimit(int increment);
		virtual void setSolutionLimit(long limit);
		virtual int getSolutionLimit();

		int lastSummaryIter = 0;
		int currIter = 0;
		double lastSummaryTimeStamp = 0.0;
		int lastHeaderIter = 0;
		bool isBusy = false;

	private:

		IloRangeArray cplexLazyConstrs;

	protected:

};

class HCallbackI: public IloCplex::HeuristicCallbackI
{
		IloNumVarArray cplexVars;

	private:

	public:
		IloCplex::CallbackI* duplicateCallback() const;
		HCallbackI(IloEnv env, IloNumVarArray xx2);
		void main();	// the call back function
};

class InfoCallbackI: public IloCplex::MIPInfoCallbackI
{
		IloNumVarArray cplexVars;

	private:

	public:
		IloCplex::CallbackI* duplicateCallback() const;
		InfoCallbackI(IloEnv env, IloNumVarArray xx2);
		void main();	// the call back function
};

class IncCallbackI: public IloCplex::IncumbentCallbackI
{
		IloNumVarArray cplexVars;

	private:
	public:
		IloCplex::CallbackI* duplicateCallback() const;
		IncCallbackI(IloEnv env, IloNumVarArray xx2);
		void main();
};

class CtCallbackI: public IloCplex::LazyConstraintCallbackI
{
		IloNumVarArray cplexVars;

		TaskBase *tSelectPrimNLP;
		bool isMinimization = true;
		int cbCalls = 0;
		int lastNumAddedHyperplanes = 0;

		TaskBase *taskSelectHPPts;
		TaskUpdateNonlinearObjectiveByLinesearch *taskUpdateObjectiveByLinesearch;
		MILPSolverCplexLazyOriginalCB *cplexSolver;

		bool checkFixedNLPStrategy(SolutionPoint point);
		void printIterationReport(SolutionPoint point);

		bool checkAbsoluteObjectiveGapToleranceMet(SolutionPoint point);
		bool checkRelativeObjectiveGapToleranceMet(SolutionPoint point);
		bool checkRelativeMIPGapToleranceMet(SolutionPoint point);

		void createHyperplane(Hyperplane hyperplane);

	public:
		IloCplex::CallbackI* duplicateCallback() const;

		CtCallbackI(IloEnv env, IloNumVarArray xx2, MILPSolverCplexLazyOriginalCB *solver);
		void main();	// the call back function
};
