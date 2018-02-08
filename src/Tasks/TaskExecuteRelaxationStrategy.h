#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MILPSolver/IRelaxationStrategy.h"
#include "../MILPSolver/RelaxationStrategyStandard.h"
#include "../MILPSolver/RelaxationStrategyNone.h"

class TaskExecuteRelaxationStrategy: public TaskBase
{
	public:
		TaskExecuteRelaxationStrategy(IMILPSolver *MILPSolver);
		~TaskExecuteRelaxationStrategy();

		void run();
		virtual std::string getType();

	private:
		IRelaxationStrategy *relaxationStrategy;

		bool isInitialized;

		IMILPSolver *MILPSolver;

};

