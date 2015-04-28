/*
 * PrimalSolutionStrategyCheckPoint.h
 *
 *  Created on: Mar 4, 2015
 *      Author: alundell
 */

#pragma once
#include <vector>
#include "PrimalSolutionStrategyBase.h"

class PrimalSolutionStrategyCheckPoint: public PrimalSolutionStrategyBase
{
	public:
		virtual ~PrimalSolutionStrategyCheckPoint();
		PrimalSolutionStrategyCheckPoint();
		/*
		 virtual bool checkPoints(std::vector<std::vector<double>> pts, std::string caller);
		 virtual bool checkPoint(std::vector<double> pt, std::string caller);
		 virtual bool checkPointOriginal(std::vector<double> pt, std::string caller);

		 virtual bool checkPoint(std::vector<double> pt, E_PrimalSolutionSource source);
		 */
		virtual bool runStrategy();
};
