/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanePointsSolution.h"

TaskSelectHyperplanePointsSolution::TaskSelectHyperplanePointsSolution(EnvironmentPtr envPtr): TaskBase(envPtr)
{
}

TaskSelectHyperplanePointsSolution::~TaskSelectHyperplanePointsSolution()
{
}

void TaskSelectHyperplanePointsSolution::run()
{
    this->run(env->process->getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsSolution::run(std::vector<SolutionPoint> solPoints)
{
    env->process->startTimer("DualCutGenerationRootSearch");

    int addedHyperplanes = 0;

    auto currIter = env->process->getCurrentIteration(); // The unsolved new iteration

    auto originalProblem = env->process->originalProblem;

    auto constrSelFactor = env->settings->getDoubleSetting("ECP.ConstraintSelectionFactor", "Dual");

    for (int i = 0; i < solPoints.size(); i++)
    {
        auto tmpMostDevConstrs = originalProblem->getMostDeviatingConstraints(solPoints.at(i).point, constrSelFactor);

        for (int j = 0; j < tmpMostDevConstrs.size(); j++)
        {
            if (addedHyperplanes >= env->settings->getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual"))
            {
                env->process->stopTimer("DualCutGenerationRootSearch");
                return;
            }

            if (tmpMostDevConstrs.at(j).value < 0)
            {
                env->output->outputWarning("LP point is in the interior!");
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

                env->process->hyperplaneWaitingList.push_back(hyperplane);

                addedHyperplanes++;
            }
        }
    }

    env->process->stopTimer("DualCutGenerationRootSearch");
}

std::string TaskSelectHyperplanePointsSolution::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
