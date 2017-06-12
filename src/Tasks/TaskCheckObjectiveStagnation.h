/*
 * TaskCheckObjectiveStagnation.h
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

class TaskCheckObjectiveStagnation: public TaskBase
{
	public:
		TaskCheckObjectiveStagnation(std::string taskIDTrue);
		virtual ~TaskCheckObjectiveStagnation();

		virtual void run();
		virtual std::string getType();

	private:
		std::string taskIDIfTrue;
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
