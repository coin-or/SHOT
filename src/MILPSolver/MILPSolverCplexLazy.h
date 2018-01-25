#pragma once
#include "IMILPSolver.h"
#include "MILPSolverBase.h"
#include "MILPSolverCallbackBase.h"

#include <functional>
#include <thread>

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wignored-attributes"
#endif
#include "ilcplex/ilocplex.h"
#ifdef __GNUC__
#pragma GCC diagnostic warning "-Wignored-attributes"
#endif


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

	private:

	protected:

};

class CplexCallback: public IloCplex::Callback::Function, public MILPSolverCallbackBase
{
	private:

		std::mutex callbackMutex;
		/* Empty constructor is forbidden. */
		CplexCallback();

		/* Copy constructor is forbidden. */
		CplexCallback(const CplexCallback &tocopy);

		IloNumVarArray cplexVars;
		IloEnv cplexEnv;

		//void printIterationReport(SolutionPoint point, const IloCplex::Callback::Context& context);

		void createHyperplane(Hyperplane hyperplane, const IloCplex::Callback::Context& context);
		void createIntegerCut(std::vector<int> binaryIndexes, const IloCplex::Callback::Context& context);

	public:
		/* Constructor with data */
		CplexCallback(const IloNumVarArray &vars, const IloEnv &env);

		void addLazyConstraint(std::vector<SolutionPoint> candidatePoints, const IloCplex::Callback::Context &context);

		// This is the function that we have to implement and that CPLEX will call
		// during the solution process at the places that we asked for.
		virtual void invoke(const IloCplex::Callback::Context &context);

		/// Destructor
		virtual ~CplexCallback();

};