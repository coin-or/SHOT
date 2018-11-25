/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/
#include "TaskSelectHyperplanePointsECP.h"

namespace SHOT
{

TaskSelectHyperplanePointsECP::TaskSelectHyperplanePointsECP(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->process->startTimer("DualCutGenerationRootSearch");
    env->process->stopTimer("DualCutGenerationRootSearch");
}

TaskSelectHyperplanePointsECP::~TaskSelectHyperplanePointsECP()
{
}

void TaskSelectHyperplanePointsECP::run()
{
    this->run(env->process->getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsECP::run(std::vector<SolutionPoint> solPoints)
{
    env->process->startTimer("DualCutGenerationRootSearch");

    int addedHyperplanes = 0;
    auto currIter = env->process->getCurrentIteration(); // The unsolved new iteration

    auto constraintSelectionFactor = env->settings->getDoubleSetting("HyperplaneCuts.ConstraintSelectionFactor", "Dual");
    bool useUniqueConstraints = env->settings->getBoolSetting("ESH.Linesearch.UniqueConstraints", "Dual");

    int maxHyperplanesPerIter = env->settings->getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual");
    double constraintMaxSelectionFactor = env->settings->getDoubleSetting("HyperplaneCuts.MaxConstraintFactor", "Dual");

    // Contains boolean array that indicates if a constraint has been added or not
    std::vector<bool> hyperplaneAddedToConstraint(env->reformulatedProblem->properties.numberOfNumericConstraints, false);

    for (int i = 0; i < solPoints.size(); i++)
    {
        auto numericConstraintValues = env->reformulatedProblem->getFractionOfDeviatingNonlinearConstraints(solPoints.at(i).point, 0.0, constraintSelectionFactor);

        for (auto &NCV : numericConstraintValues)
        {
            if (addedHyperplanes >= maxHyperplanesPerIter)
            {
                env->process->stopTimer("DualCutGenerationRootSearch");
                return;
            }

            // Do not add hyperplane if one has been added for this constraint already
            if (useUniqueConstraints && hyperplaneAddedToConstraint.at(NCV.constraint->index))
                continue;

            // Do not add hyperplane if there are numerical errors
            if (isnan(NCV.error) || isnan(NCV.normalizedValue))
            {
                continue;
            }

            // Do not add hyperplane if constraint value is much less than largest
            if (NCV.error < constraintMaxSelectionFactor * numericConstraintValues.at(0).error)
            {
                continue;
            }

            Hyperplane hyperplane;
            hyperplane.sourceConstraint = NCV.constraint;
            hyperplane.sourceConstraintIndex = NCV.constraint->index;
            hyperplane.generatedPoint = solPoints.at(i).point;

            if (solPoints.at(i).isRelaxedPoint)
            {
                hyperplane.source = E_HyperplaneSource::MIPCallbackRelaxed;
            }
            else if (i == 0 && currIter->isMIP())
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
            hyperplaneAddedToConstraint.at(NCV.constraint->index) = true;
        }
    }

    if (addedHyperplanes == 0)
    {
        env->output->outputInfo("     All nonlinear constraints fulfilled, so no constraint cuts added.");
    }

    env->process->stopTimer("DualCutGenerationRootSearch");
}

std::string TaskSelectHyperplanePointsECP::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT