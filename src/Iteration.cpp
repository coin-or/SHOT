/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Iteration.h"
#include "Results.h"
#include "Settings.h"

namespace SHOT
{

Iteration::Iteration(EnvironmentPtr envPtr)
{
    env = envPtr;
    this->iterationNumber = env->results->getNumberOfIterations() + 1;

    this->numHyperplanesAdded = 0;

    if(env->results->getNumberOfIterations() == 0)
        this->totNumHyperplanes = 0;
    else if(env->settings->getSetting<bool>("TreeStrategy.Multi.Reinitialize", "Dual"))
        this->totNumHyperplanes = 0;
    else
        this->totNumHyperplanes
            = env->results->iterations.at(env->results->getNumberOfIterations() - 1)->totNumHyperplanes;

    this->maxDeviation = SHOT_DBL_MAX;
    this->boundaryDistance = SHOT_DBL_MAX;

    this->objectiveValue = NAN;
    this->MIPSolutionLimitUpdated = false;
    this->solutionStatus = E_ProblemSolutionStatus::None;

    currentObjectiveBounds.first = env->results->getCurrentDualBound();
    currentObjectiveBounds.second = env->results->getPrimalBound();
}

Iteration::~Iteration()
{
    solutionPoints.clear();
    constraintDeviations.clear();
    hyperplanePoints.clear();
}

bool Iteration::isMIP() { return (this->isDualProblemDiscrete); }

SolutionPoint Iteration::getSolutionPointWithSmallestDeviation()
{
    double tmpVal = SHOT_DBL_MIN;
    int tmpIdx = 0;

    for(size_t i = 0; i < solutionPoints.size(); i++)
    {
        if(solutionPoints.at(i).maxDeviation.value > tmpVal)
        {
            tmpIdx = i;
            tmpVal = solutionPoints.at(i).maxDeviation.value;
        }
    }

    return (solutionPoints.at(tmpIdx));
}

int Iteration::getSolutionPointWithSmallestDeviationIndex()
{
    double tmpVal = SHOT_DBL_MIN;
    int tmpIdx = 0;

    for(size_t i = 0; i < solutionPoints.size(); i++)
    {
        if(solutionPoints.at(i).maxDeviation.value > tmpVal)
        {
            tmpIdx = i;
            tmpVal = solutionPoints.at(i).maxDeviation.value;
        }
    }

    return (tmpIdx);
}
} // namespace SHOT