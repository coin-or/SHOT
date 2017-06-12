/*
 * TaskGoto.h
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#pragma once

#include "TaskBase.h"

#include "../ProcessInfo.h"

class TaskGoto: public TaskBase
{
	public:
		TaskGoto(std::string taskID);
		virtual ~TaskGoto();

		virtual void run();
		virtual std::string getType();

	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

		std::string gotoTaskID;
};

