/*
 * PrimalSolutionStrategyBase.h

 *
 *  Created on: Mar 4, 2015
 *      Author: alundell
 */

#pragma once
#include "../PrimalSolutionStrategy/IPrimalSolutionStrategy.h"
#include "SHOTSettings.h"
#include "../ProcessInfo.h"
#include "../OptProblems/OptProblemOriginal.h"

class PrimalSolutionStrategyBase: public IPrimalSolutionStrategy
{
	public:
		PrimalSolutionStrategyBase();
		virtual ~PrimalSolutionStrategyBase();

	protected:
		SHOTSettings::Settings *settings;
		//ProcessInfo *processInfo;

	private:

};
