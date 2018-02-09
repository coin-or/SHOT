#pragma once
#include "../Tasks/TaskUpdateNonlinearObjectiveByLinesearch.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromLinesearch.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromNLP.h"
#include "../Tasks/TaskSelectHyperplanePointsLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsIndividualLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsSolution.h"

class MIPSolverCallbackBase
{
	public:

	private:
    
	protected:
		int cbCalls = 0;
		bool isMinimization = true;
		int lastNumAddedHyperplanes = 0;
		double lastUpdatedPrimal;

		int maxIntegerRelaxedHyperplanes = 0;

		int lastSummaryIter = 0;
		double lastSummaryTimeStamp = 0.0;
		int lastHeaderIter = 0;

		TaskBase *tSelectPrimNLP;
		TaskBase *taskSelectHPPts;
		TaskUpdateNonlinearObjectiveByLinesearch *taskUpdateObjectiveByLinesearch;
		TaskSelectPrimalCandidatesFromLinesearch *taskSelectPrimalSolutionFromLinesearch;

		bool checkFixedNLPStrategy(SolutionPoint point);

		bool checkIterationLimit();

		void addLazyConstraint(std::vector<SolutionPoint> candidatePoints);

		void printIterationReport(SolutionPoint solution, std::string threadId, std::string bestBound, std::string openNodes);

};
