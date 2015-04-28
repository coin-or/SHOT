/*
 * TaskCheckTimeLimit.h
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

class TaskCheckTimeLimit: public TaskBase
{
	public:
		TaskCheckTimeLimit(std::string taskIDTrue);
		virtual ~TaskCheckTimeLimit();

		virtual void run();

	private:
		std::string taskIDIfTrue;
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
