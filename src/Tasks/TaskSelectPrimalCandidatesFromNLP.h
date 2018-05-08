/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"
#include "../PrimalSolutionStrategy/PrimalSolutionStrategyFixedNLP.h"

class TaskSelectPrimalCandidatesFromNLP : public TaskBase
{
  public:
	TaskSelectPrimalCandidatesFromNLP(EnvironmentPtr envPtr);
	virtual ~TaskSelectPrimalCandidatesFromNLP();
	virtual void run();
	virtual std::string getType();

  private:
	PrimalSolutionStrategyFixedNLP *primalStrategyFixedNLP;
};
