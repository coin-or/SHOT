#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../LinesearchMethod/LinesearchMethodBoost.h"
#include "../LinesearchMethod/LinesearchMethodBisection.h"

class TaskInitializeLinesearch: public TaskBase
{
	public:
		TaskInitializeLinesearch();
		virtual ~TaskInitializeLinesearch();

		virtual void run();

		virtual std::string getType();
	private:
		ILinesearchMethod *linesearchMethod;

};
