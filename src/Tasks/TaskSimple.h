#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"
#include <functional>

class TaskSimple: public TaskBase
{
	public:
		TaskSimple(std::function<bool()> taskFunction);
		TaskSimple();
		~TaskSimple();

		void setFunction(std::function<bool()> taskFunction);

		virtual void run();

	private:
		std::function<bool()> task;

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

};
