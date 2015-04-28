/*
 * TaskTerminate.h
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#pragma once

#include <TaskBase.h>
#include "../ProcessInfo.h"

class TaskTerminate: public TaskBase
{
	public:
		TaskTerminate();
		virtual ~TaskTerminate();

		virtual void run();

	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};

