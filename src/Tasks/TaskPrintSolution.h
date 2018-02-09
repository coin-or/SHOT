#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"
#include "../MIPSolver/IMIPSolver.h"
class TaskPrintSolution: public TaskBase
{
	public:
		TaskPrintSolution();
		~TaskPrintSolution();

		virtual void run();
		virtual std::string getType();

	private:

};

