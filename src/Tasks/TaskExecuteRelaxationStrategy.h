#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

#include "../MILPSolver/IRelaxationStrategy.h"
#include "../MILPSolver/RelaxationStrategyStandard.h"
#include "../MILPSolver/RelaxationStrategyAdaptive.h"
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

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

		bool isInitialized;

		IMILPSolver *MILPSolver;

};

