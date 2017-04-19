#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

#include "../MILPSolver/IMILPSolver.h"

class TaskPresolve: public TaskBase
{
	public:
		TaskPresolve();
		virtual ~TaskPresolve();

		virtual void run();
		virtual std::string getType();
	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
		bool isPresolved = false;
};

