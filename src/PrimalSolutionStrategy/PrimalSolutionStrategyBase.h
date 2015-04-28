/*
 * PrimalSolutionStrategyBase.h

 *
 *  Created on: Mar 4, 2015
 *      Author: alundell
 */

#pragma once
#include <PrimalSolutionStrategy/IPrimalSolutionStrategy.h>
#include "SHOTSettings.h"
#include "../ProcessInfo.h"
#include "../OptProblems/OptProblemOriginal.h"

class PrimalSolutionStrategyBase: public IPrimalSolutionStrategy
{
	public:
		PrimalSolutionStrategyBase();
		virtual ~PrimalSolutionStrategyBase();

		//virtual bool checkPoint(std::vector<double> pt, E_PrimalSolutionSource source);
		//virtual bool checkPoints(std::vector<std::vector<double>> pts, E_PrimalSolutionSource source);

		virtual bool checkPoint(PrimalSolution primalSol);
		virtual bool checkPoints(std::vector<PrimalSolution> primalSols);

	protected:
		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;

	private:

};
