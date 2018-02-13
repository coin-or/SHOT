#pragma once

#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../OptProblems/OptProblemOriginal.h"

class TaskUpdateInteriorPoint: public TaskBase
{
	public:
		TaskUpdateInteriorPoint();
		virtual ~TaskUpdateInteriorPoint();
		virtual void run();
		virtual std::string getType();

	private:

};

