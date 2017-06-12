/*
 * TaskCheckIterationLimit.h
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

class TaskCheckIterationLimit: public TaskBase
{
	public:
		TaskCheckIterationLimit(std::string taskIDTrue);
		virtual ~TaskCheckIterationLimit();

		virtual void run();
		virtual std::string getType();

	private:
		std::string taskIDIfTrue;
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
