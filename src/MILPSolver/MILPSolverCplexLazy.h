#pragma once
#include "IMILPSolver.h"
#include "MILPSolverBase.h"

#include <functional>
#include <thread>

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
#include "../Tasks/TaskSelectPrimalCandidatesFromLinesearch.h"

class MILPSolverCplexLazy: public MILPSolverCplex
{
	public:

		MILPSolverCplexLazy();
		virtual ~MILPSolverCplexLazy();

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
		//bool isBusy = false;

	private:

		//IloRangeArray cplexLazyConstrs;

	protected:

};

class CplexCallback: public IloCplex::Callback::Function
{
	private:
		/* Empty constructor is forbidden. */
		CplexCallback();

		/* Copy constructor is forbidden. */
		CplexCallback(const CplexCallback &tocopy);

		bool isMinimization = true;
		int lastNumAddedHyperplanes = 0;

		double lastUpdatedPrimal;

		IloNumVarArray cplexVars;

		TaskBase *tSelectPrimNLP;
		TaskBase *taskSelectHPPts;
		TaskUpdateNonlinearObjectiveByLinesearch *taskUpdateObjectiveByLinesearch;
		TaskSelectPrimalCandidatesFromLinesearch *taskSelectPrimalSolutionFromLinesearch;

		bool checkFixedNLPStrategy(SolutionPoint point);
		void printIterationReport(SolutionPoint point, const IloCplex::Callback::Context& context);

		bool checkAbsoluteObjectiveGapToleranceMet(const IloCplex::Callback::Context& context);
		bool checkRelativeObjectiveGapToleranceMet(const IloCplex::Callback::Context& context);
		//bool checkRelativeMIPGapToleranceMet(SolutionPoint point, const IloCplex::Callback::Context& context);

		void createHyperplane(Hyperplane hyperplane, const IloCplex::Callback::Context& context);
		void createIntegerCut(std::vector<int> binaryIndexes, const IloCplex::Callback::Context& context);
	public:
		/* Constructor with data */
		CplexCallback(const IloNumVarArray &vars);

		void addLazyConstraint(std::vector<SolutionPoint> candidatePoints, const IloCplex::Callback::Context &context);
		//void heuristicCallback(const IloCplex::Callback::Context &context);

		// This is the function that we have to implement and that CPLEX will call
		// during the solution process at the places that we asked for.
		virtual void invoke(const IloCplex::Callback::Context &context);

		/// Destructor
		virtual ~CplexCallback();

};

/*
 class HCallbackI2: public IloCplex::HeuristicCallbackI
 {
 IloNumVarArray cplexVars;

 private:

 public:
 IloCplex::CallbackI* duplicateCallback() const;
 HCallbackI2(IloEnv env, IloNumVarArray xx2);
 void main();	// the call back function
 };

 class InfoCallbackI2: public IloCplex::MIPInfoCallbackI
 {
 IloNumVarArray cplexVars;

 private:

 public:
 IloCplex::CallbackI* duplicateCallback() const;
 InfoCallbackI2(IloEnv env, IloNumVarArray xx2);
 void main();	// the call back function
 };

 class IncCallbackI2: public IloCplex::IncumbentCallbackI
 {
 IloNumVarArray cplexVars;

 private:
 public:
 IloCplex::CallbackI* duplicateCallback() const;
 IncCallbackI2(IloEnv env, IloNumVarArray xx2);
 void main();
 };

 class CtCallbackI2: public IloCplex::LazyConstraintCallbackI
 {
 IloNumVarArray cplexVars;

 TaskBase *tSelectPrimNLP;
 bool isMinimization = true;
 int cbCalls = 0;
 int lastNumAddedHyperplanes = 0;

 TaskBase *taskSelectHPPts;
 TaskUpdateNonlinearObjectiveByLinesearch *taskUpdateObjectiveByLinesearch;
 MILPSolverCplexLazy *cplexSolver;

 bool checkFixedNLPStrategy(SolutionPoint point);
 void printIterationReport(SolutionPoint point);

 bool checkAbsoluteObjectiveGapToleranceMet(SolutionPoint point);
 bool checkRelativeObjectiveGapToleranceMet(SolutionPoint point);
 bool checkRelativeMIPGapToleranceMet(SolutionPoint point);

 void createHyperplane(Hyperplane hyperplane);

 public:
 IloCplex::CallbackI* duplicateCallback() const;

 CtCallbackI2(IloEnv env, IloNumVarArray xx2, MILPSolverCplexLazy *solver);
 void main();	// the call back function
 };*/
