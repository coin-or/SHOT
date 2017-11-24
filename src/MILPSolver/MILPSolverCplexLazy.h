#pragma once
#include "IMILPSolver.h"
#include "MILPSolverBase.h"

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

class MILPSolverCplexLazy: public MILPSolverCplex
{
	public:

		MILPSolverCplexLazy();
		virtual ~MILPSolverCplexLazy();

		virtual void checkParameters();

		virtual void initializeSolverSettings();

		virtual int addLinearConstraint(std::vector<IndexValuePair> elements, double constant)
		{
			return (addLinearConstraint(elements, constant, false));
		}
		virtual int addLinearConstraint(std::vector<IndexValuePair> elements, double constant, bool isGreaterThan);

		virtual E_ProblemSolutionStatus solveProblem();

		virtual int increaseSolutionLimit(int increment);
		virtual void setSolutionLimit(long limit);
		virtual int getSolutionLimit();

	private:

		IloRangeArray cplexLazyConstrs;

	protected:

};
