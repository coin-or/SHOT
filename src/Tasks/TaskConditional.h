#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"
#include <functional>
class TaskConditional: public TaskBase
{
	public:
		TaskConditional(std::function<bool()> conditionFunct, TaskBase *taskIfTrue, TaskBase *taskIfFalse);
		TaskConditional();
		~TaskConditional();

		void setTaskIfTrue(TaskBase * task);
		void setTaskIfFalse(TaskBase * task);
		void setCondition(std::function<bool()> conditionFunct);

		virtual void run();
		virtual std::string getType();

	private:
		std::function<bool()> condition;

		TaskBase *taskIfTrue;
		TaskBase *taskIfFalse;
		bool taskFalseIsSet;
	protected:
		SHOTSettings::Settings *settings;
		//ProcessInfo *processInfo;
};
