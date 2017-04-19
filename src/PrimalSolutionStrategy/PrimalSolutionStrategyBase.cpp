/*
 * PrimalSolutionStrategyBase.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: alundell
 */

#include <PrimalSolutionStrategy/PrimalSolutionStrategyBase.h>

PrimalSolutionStrategyBase::PrimalSolutionStrategyBase()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

PrimalSolutionStrategyBase::~PrimalSolutionStrategyBase()
{
}
