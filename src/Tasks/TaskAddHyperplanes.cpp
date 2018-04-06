/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskAddHyperplanes.h"

TaskAddHyperplanes::TaskAddHyperplanes(IMIPSolver *MIPSolver)
{
    ProcessInfo::getInstance().startTimer("DualStrategy");
    itersWithoutAddedHPs = 0;

    this->MIPSolver = MIPSolver;
    ProcessInfo::getInstance().stopTimer("DualStrategy");
}

TaskAddHyperplanes::~TaskAddHyperplanes()
{
}

void TaskAddHyperplanes::run()
{
    ProcessInfo::getInstance().startTimer("DualStrategy");
    this->MIPSolver = MIPSolver;
    auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

    if (!currIter->isMIP() || !Settings::getInstance().getBoolSetting("HyperplaneCuts.Delay", "Dual") || !currIter->MIPSolutionLimitUpdated || itersWithoutAddedHPs > 5)
    {
        int addedHyperplanes = 0;

        for (int k = ProcessInfo::getInstance().hyperplaneWaitingList.size(); k > 0; k--)
        {
            if (addedHyperplanes >= Settings::getInstance().getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual"))
                break;

            auto tmpItem = ProcessInfo::getInstance().hyperplaneWaitingList.at(k - 1);

            if (tmpItem.source == E_HyperplaneSource::PrimalSolutionSearchInteriorObjective)
            {
                MIPSolver->createInteriorHyperplane(tmpItem);
            }
            else
            {
                MIPSolver->createHyperplane(tmpItem);

                ProcessInfo::getInstance().addedHyperplanes.push_back(tmpItem);

                addedHyperplanes++;
            }
        }

        ProcessInfo::getInstance().hyperplaneWaitingList.clear();
        itersWithoutAddedHPs = 0;
    }
    else
    {
        itersWithoutAddedHPs++;
    }

    ProcessInfo::getInstance().stopTimer("DualStrategy");
}

std::string TaskAddHyperplanes::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
