/*
 * PrimalSolutionStrategyCheckPoint.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: alundell
 */

#include <PrimalSolutionStrategyCheckPoint.h>

PrimalSolutionStrategyCheckPoint::~PrimalSolutionStrategyCheckPoint()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

PrimalSolutionStrategyCheckPoint::PrimalSolutionStrategyCheckPoint()
{
	// TODO Auto-generated constructor stub
}

/*
 bool PrimalSolutionStrategyCheckPoint::checkPoints(std::vector<std::vector<double> > pts, std::string caller)
 {
 return PrimalSolutionStrategyBase::checkPoints(pts, caller);
 }

 bool PrimalSolutionStrategyCheckPoint::checkPoint(std::vector<double> pt, std::string caller)
 {
 return PrimalSolutionStrategyBase::checkPoint(pt, caller);
 }

 bool PrimalSolutionStrategyCheckPoint::checkPoint(std::vector<double> pt, E_PrimalSolutionSource source)
 {
 return PrimalSolutionStrategyBase::checkPoint(pt, source);
 }

 bool PrimalSolutionStrategyCheckPoint::checkPointOriginal(std::vector<double> pt, std::string caller)
 {
 return PrimalSolutionStrategyBase::checkPointOriginal(pt, caller);
 }*/

bool PrimalSolutionStrategyCheckPoint::runStrategy()
{
	return false;
}
