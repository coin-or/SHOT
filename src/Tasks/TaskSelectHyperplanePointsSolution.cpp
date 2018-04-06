/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanePointsSolution.h"

TaskSelectHyperplanePointsSolution::TaskSelectHyperplanePointsSolution()
{
}

TaskSelectHyperplanePointsSolution::~TaskSelectHyperplanePointsSolution()
{
}

void TaskSelectHyperplanePointsSolution::run()
{
    this->run(ProcessInfo::getInstance().getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsSolution::run(vector<SolutionPoint> solPoints)
{
    ProcessInfo::getInstance().startTimer("DualCutGenerationRootSearch");

    int addedHyperplanes = 0;

    auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

    auto originalProblem = ProcessInfo::getInstance().originalProblem;

    auto constrSelFactor = Settings::getInstance().getDoubleSetting("ECP.ConstraintSelectionFactor", "Dual");

    for (int i = 0; i < solPoints.size(); i++)
    {
        auto tmpMostDevConstrs = originalProblem->getMostDeviatingConstraints(solPoints.at(i).point, constrSelFactor);

        for (int j = 0; j < tmpMostDevConstrs.size(); j++)
        {
            if (addedHyperplanes >= Settings::getInstance().getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual"))
            {
                ProcessInfo::getInstance().stopTimer("DualCutGenerationRootSearch");
                return;
            }

            if (tmpMostDevConstrs.at(j).value < 0)
            {
                Output::getInstance().outputWarning("LP point is in the interior!");
            }
            else
            {
                Hyperplane hyperplane;
                hyperplane.sourceConstraintIndex = tmpMostDevConstrs.at(j).idx;
                hyperplane.generatedPoint = solPoints.at(i).point;

                if (i == 0 && currIter->isMIP())
                {
                    hyperplane.source = E_HyperplaneSource::MIPOptimalSolutionPoint;
                }
                else if (currIter->isMIP())
                {
                    hyperplane.source = E_HyperplaneSource::MIPSolutionPoolSolutionPoint;
                }
                else
                {
                    hyperplane.source = E_HyperplaneSource::LPRelaxedSolutionPoint;
                }

                ProcessInfo::getInstance().hyperplaneWaitingList.push_back(hyperplane);

                addedHyperplanes++;
            }
        }
    }

    ProcessInfo::getInstance().stopTimer("DualCutGenerationRootSearch");
}

std::string TaskSelectHyperplanePointsSolution::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
