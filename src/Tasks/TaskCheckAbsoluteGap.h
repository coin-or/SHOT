/*
 * TaskCheckAbsoluteGap.h
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#pragma once
#include <TaskBase.h>
#include "../ProcessInfo.h"

class TaskCheckAbsoluteGap: public TaskBase
{
	public:
		TaskCheckAbsoluteGap(std::string taskIDTrue);
		virtual ~TaskCheckAbsoluteGap();

		virtual void run();

	private:

		std::string taskIDIfTrue;

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
