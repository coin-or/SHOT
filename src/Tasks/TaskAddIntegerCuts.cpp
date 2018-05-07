/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskAddIntegerCuts.h"

TaskAddIntegerCuts::TaskAddIntegerCuts(IMIPSolver *MIPSolver)
{
    ProcessInfo::getInstance().startTimer("DualStrategy");
    this->MIPSolver = MIPSolver;
    ProcessInfo::getInstance().stopTimer("DualStrategy");
}

TaskAddIntegerCuts::~TaskAddIntegerCuts()
{
}

void TaskAddIntegerCuts::run()
{
    ProcessInfo::getInstance().startTimer("DualStrategy");

    auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

    if (ProcessInfo::getInstance().integerCutWaitingList.size() == 0)
        return;

    if (!currIter->isMIP() || !Settings::getInstance().getBoolSetting("HyperplaneCuts.Delay", "Dual") || !currIter->MIPSolutionLimitUpdated)
    {

        for (int j = 0; j < ProcessInfo::getInstance().integerCutWaitingList.size(); j++)
        {
            auto tmpBinaryCombination = ProcessInfo::getInstance().integerCutWaitingList.at(j);
            int numOnes = tmpBinaryCombination.size();

            std::vector<IndexValuePair> elements;

            for (int i = 0; i < numOnes; i++)
            {
                IndexValuePair pair;
                pair.idx = tmpBinaryCombination.at(i);
                pair.value = 1.0;

                elements.push_back(pair);
            }

            this->MIPSolver->addLinearConstraint(elements, -(numOnes - 1.0));
            ProcessInfo::getInstance().solutionStatistics.numberOfIntegerCuts++;
        }

        Output::getInstance().outputInfo(
            "     Added " + to_string(ProcessInfo::getInstance().integerCutWaitingList.size()) + " integer cut(s).                                        ");

        ProcessInfo::getInstance().integerCutWaitingList.clear();
    }

    ProcessInfo::getInstance().stopTimer("DualStrategy");
}

std::string TaskAddIntegerCuts::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
