/*

 * SolutionStrategyNLP.h
 *
 *  Created on: Mar 30, 2015
 *      Author: alundell
 */

#pragma once

#include "ISolutionStrategy.h"
#include "../Tasks/TaskFindInteriorPoint.h"

#include "../Tasks/TaskBase.h"
#include "../Tasks/TaskSequential.h"
#include "../Tasks/TaskGoto.h"
#include "../Tasks/TaskConditional.h"

#include "../Tasks/TaskInitializeOriginalProblem.h"
#include "../Tasks/TaskPrintProblemStats.h"
#include "../Tasks/TaskInitializeIteration.h"
#include "../Tasks/TaskTerminate.h"

#include "../Tasks/TaskInitializeMILPSolver.h"
#include "../Tasks/TaskCreateMILPProblem.h"

#include "../Tasks/TaskPrintIterationHeader.h"
#include "../Tasks/TaskExecuteRelaxationStrategy.h"

#include "../Tasks/TaskPrintIterationReport.h"
#include "../Tasks/TaskPrintSolutionBoundReport.h"

#include "../Tasks/TaskSolveIteration.h"
#include "../Tasks/TaskPresolve.h"

#include "../Tasks/TaskCheckAbsoluteGap.h"
#include "../Tasks/TaskCheckIterationError.h"
#include "../Tasks/TaskCheckIterationLimit.h"
#include "../Tasks/TaskCheckObjectiveStagnation.h"
#include "../Tasks/TaskCheckConstraintTolerance.h"
#include "../Tasks/TaskCheckRelativeGap.h"
#include "../Tasks/TaskCheckTimeLimit.h"

#include "../Tasks/TaskPrintSolution.h"

#include "../Tasks/TaskInitializeLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsIndividualLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsSolution.h"
#include "../Tasks/TaskAddHyperplanes.h"

#include "../Tasks/TaskSelectPrimalCandidatesFromSolutionPool.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromLinesearch.h"

#include "../Tasks/TaskUpdateNonlinearObjectiveByLinesearch.h"

#include "SHOTSettings.h"
#include "../ProcessInfo.h"

class SolutionStrategyNLP: public ISolutionStrategy
{
	public:
		SolutionStrategyNLP(OSInstance *osInstance);
		virtual ~SolutionStrategyNLP();

		virtual bool solveProblem();
		virtual void initializeStrategy();

	protected:

		//TaskSequential *mainTask;

};

