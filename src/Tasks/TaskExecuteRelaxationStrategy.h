#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MIPSolver/IRelaxationStrategy.h"
#include "../MIPSolver/RelaxationStrategyStandard.h"
#include "../MIPSolver/RelaxationStrategyNone.h"

class TaskExecuteRelaxationStrategy: public TaskBase
{
	public:
		TaskExecuteRelaxationStrategy(IMIPSolver *MIPSolver);
		~TaskExecuteRelaxationStrategy();

		void run();
		virtual std::string getType();

	private:
		IRelaxationStrategy *relaxationStrategy;

		bool isInitialized;

		IMIPSolver *MIPSolver;

};

