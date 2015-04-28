#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"
#include "UtilityFunctions.h"
#include "../OptProblems/OptProblemOriginal.h"
#include "../OptProblems/OptProblemOriginalNonlinearObjective.h"
#include "../OptProblems/OptProblemOriginalQuadraticObjective.h"
#include "../OptProblems/OptProblemOriginalLinearObjective.h"
#include <OSInstance.h>

class TaskInitializeOriginalProblem: public TaskBase
{
	public:
		TaskInitializeOriginalProblem(OSInstance *originalInstance);
		~TaskInitializeOriginalProblem();

		void run();

	private:
		OptProblemOriginal *problem;
		OSInstance * instance;

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};

