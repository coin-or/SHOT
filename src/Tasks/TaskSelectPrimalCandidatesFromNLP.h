/*
 * TaskSelectPrimalCandidatesFromNLP.h
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#pragma once

#include <TaskBase.h>
#include "../ProcessInfo.h"
#include "../PrimalSolutionStrategy/PrimalSolutionStrategyFixedNLP.h"

class TaskSelectPrimalCandidatesFromNLP: public TaskBase
{
	public:
		TaskSelectPrimalCandidatesFromNLP();
		virtual ~TaskSelectPrimalCandidatesFromNLP();
		virtual void run();
		virtual std::string getType();
		//void setFixedPoint(std::vector<double> fixedPt);
	private:
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

		PrimalSolutionStrategyFixedNLP *primalStrategyFixedNLP;
};

