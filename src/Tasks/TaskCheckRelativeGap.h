/*
 * TaskCheckRelativeGap.h
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

class TaskCheckRelativeGap: public TaskBase
{
	public:
		TaskCheckRelativeGap(std::string taskIDTrue);
		virtual ~TaskCheckRelativeGap();

		virtual void run();

	private:
		std::string taskIDIfTrue;
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
