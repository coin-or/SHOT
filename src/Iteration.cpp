/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "Iteration.h"

namespace SHOT
{

Iteration::Iteration(EnvironmentPtr envPtr)
{
    env = envPtr;
    this->iterationNumber = env->process->iterations.size() + 1;

    this->numHyperplanesAdded = 0;

    if (env->process->iterations.size() == 0)
        this->totNumHyperplanes = 0;
    else if (env->settings->getBoolSetting("TreeStrategy.Multi.Reinitialize", "Dual"))
        this->totNumHyperplanes = 0;
    else
        this->totNumHyperplanes = env->process->iterations.at(env->process->iterations.size() - 1).totNumHyperplanes;

    this->maxDeviation = OSDBL_MAX;
    this->boundaryDistance = OSDBL_MAX;

    this->objectiveValue = NAN;
    this->MIPSolutionLimitUpdated = false;
    this->solutionStatus = E_ProblemSolutionStatus::None;

    currentObjectiveBounds.first = env->process->getDualBound();
    currentObjectiveBounds.second = env->process->getPrimalBound();

    if (env->process->relaxationStrategy)
    {
        this->type = env->process->relaxationStrategy->getProblemType();
    }
    else
    {
        switch (static_cast<E_SolutionStrategy>(env->process->usedSolutionStrategy))
        {
        case (E_SolutionStrategy::MIQCQP):
            this->type = E_IterationProblemType::MIP;
            break;
        case (E_SolutionStrategy::MIQP):
            this->type = E_IterationProblemType::MIP;
            break;
        case (E_SolutionStrategy::NLP):
            this->type = E_IterationProblemType::Relaxed;
            break;
        case (E_SolutionStrategy::SingleTree):
            this->type = E_IterationProblemType::MIP;
            break;
        default:
            this->type = E_IterationProblemType::MIP;
            break;
        }
    }
}

Iteration::~Iteration()
{
    solutionPoints.clear();
    constraintDeviations.clear();
    hyperplanePoints.clear();
}

bool Iteration::isMIP()
{
    return (this->type == E_IterationProblemType::MIP);
}

SolutionPoint Iteration::getSolutionPointWithSmallestDeviation()
{
    double tmpVal = -OSDBL_MAX;
    int tmpIdx = 0;

    for (int i = 0; i < solutionPoints.size(); i++)
    {
        if (solutionPoints.at(i).maxDeviation.value > tmpVal)
        {
            tmpIdx = i;
            tmpVal = solutionPoints.at(i).maxDeviation.value;
        }
    }

    return (solutionPoints.at(tmpIdx));
}

int Iteration::getSolutionPointWithSmallestDeviationIndex()
{
    double tmpVal = -OSDBL_MAX;
    int tmpIdx = 0;

    for (int i = 0; i < solutionPoints.size(); i++)
    {
        if (solutionPoints.at(i).maxDeviation.value > tmpVal)
        {
            tmpIdx = i;
            tmpVal = solutionPoints.at(i).maxDeviation.value;
        }
    }

    return (tmpIdx);
}
} // namespace SHOT