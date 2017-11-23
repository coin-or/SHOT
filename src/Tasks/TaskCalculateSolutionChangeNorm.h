/*
 * TaskCalculateSolutionChangeNorm.h
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#pragma once

#include "TaskBase.h"
#include "../ProcessInfo.h"

class TaskCalculateSolutionChangeNorm: public TaskBase
{
	public:
		TaskCalculateSolutionChangeNorm();
		virtual ~TaskCalculateSolutionChangeNorm();

		virtual void run();

		virtual std::string getType();
	private:

};

