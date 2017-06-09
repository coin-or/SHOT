/*
 * IPrimalSolutionStrategy.h
 *
 *  Created on: Mar 4, 2015
 *      Author: alundell
 */

#pragma once

class IPrimalSolutionStrategy {
public:
	//IPrimalSolutionStrategy();
	virtual ~IPrimalSolutionStrategy();

	virtual bool runStrategy() = 0;
};
