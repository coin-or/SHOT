/*
 * TaskCheckIterationError.h
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

class TaskCheckIterationError: public TaskBase
{
	public:
		TaskCheckIterationError(std::string taskIDTrue);
		virtual ~TaskCheckIterationError();

		virtual void run();

		virtual std::string getType();
	private:
		std::string taskIDIfTrue;
		SHOTSettings::Settings *settings;
		//ProcessInfo *processInfo;
};
