/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromNLP.h"

TaskSelectPrimalCandidatesFromNLP::TaskSelectPrimalCandidatesFromNLP()
{
    ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
    ProcessInfo::getInstance().startTimer("PrimalBoundSearchNLP");

    primalStrategyFixedNLP = new PrimalSolutionStrategyFixedNLP();

    ProcessInfo::getInstance().stopTimer("PrimalBoundSearchNLP");
    ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
}

TaskSelectPrimalCandidatesFromNLP::~TaskSelectPrimalCandidatesFromNLP()
{
    delete primalStrategyFixedNLP;
}

void TaskSelectPrimalCandidatesFromNLP::run()
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    if (currIter->isMIP() && ProcessInfo::getInstance().getRelativeObjectiveGap() > 1e-10)
    {
        ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
        ProcessInfo::getInstance().startTimer("PrimalBoundSearchNLP");

        primalStrategyFixedNLP->runStrategy();

        ProcessInfo::getInstance().stopTimer("PrimalBoundSearchNLP");
        ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
    }
}

std::string TaskSelectPrimalCandidatesFromNLP::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
