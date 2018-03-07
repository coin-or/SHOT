/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "Iteration.h"

Iteration::Iteration()
{
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
