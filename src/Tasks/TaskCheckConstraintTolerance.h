/*
 * TaskCheckConstraintTolerance.h
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"

class TaskCheckConstraintTolerance: public TaskBase
{
	public:
		TaskCheckConstraintTolerance(std::string taskIDTrue);
		virtual ~TaskCheckConstraintTolerance();

		virtual void run();

		virtual std::string getType();
	private:
		std::string taskIDIfTrue;

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};
