/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "LinesearchMethodBisection.h"

using namespace SHOT;

LinesearchMethodBisection::LinesearchMethodBisection(EnvironmentPtr envPtr)
{
    env = envPtr;
}

LinesearchMethodBisection::~LinesearchMethodBisection()
{
}

std::pair<VectorDouble, VectorDouble> LinesearchMethodBisection::findZero(VectorDouble ptA,
                                                                                      VectorDouble ptB, int Nmax, double lambdaTol, double constrTol)
{
    bool validNewPt = false;
    try
    {
        int length = ptA.size();
        bool validB = env->model->originalProblem->isConstraintsFulfilledInPoint(ptB);

        VectorDouble ptNew(length);
        VectorDouble ptNew2(length);

        double a = 0;
        double b = 1;
        double c;
        int n = 1;

        while (n <= Nmax)
        {
            c = (a + b) / 2.0;
            for (int i = 0; i < length; i++)
            {
                ptNew.at(i) = c * ptA.at(i) + (1 - c) * ptB.at(i);
                ptNew2.at(i) = c * ptB.at(i) + (1 - c) * ptA.at(i);
            }

            validNewPt = env->model->originalProblem->isConstraintsFulfilledInPoint(ptNew);

            if ((b - a) / 2 < lambdaTol)
            {
                break;
            }

            n++;

            if ((validNewPt && validB) || (!validNewPt && !validB))
            {
                a = c;
                validB = validNewPt;
            }
            else
            {
                b = c;
            }
        }

        env->output->outputInfo("Linesearch completed in " + std::to_string(n) + "iterations.");
        if (!validNewPt)
        {
            std::pair<VectorDouble, VectorDouble> tmpPair(ptNew2, ptNew);
            return (tmpPair);
        }
        else
        {
            std::pair<VectorDouble, VectorDouble> tmpPair(ptNew, ptNew2);
            return (tmpPair);
        }
    }
    catch (...)
    {
        env->output->outputError("Error while doing linesearch.");

        if (!env->model->originalProblem->isConstraintsFulfilledInPoint(ptA))
            //Returns the NLP point if not on the interior

            if (!env->model->originalProblem->isConstraintsFulfilledInPoint(ptA))
            {

                std::pair<VectorDouble, VectorDouble> tmpPair(ptB, ptA);
                return (tmpPair);
            }

        std::pair<VectorDouble, VectorDouble> tmpPair(ptA, ptB);
        return (tmpPair);
    }
}

std::pair<VectorDouble, VectorDouble> LinesearchMethodBisection::findZero(VectorDouble ptA,
                                                                                      VectorDouble ptB, int Nmax, double lambdaTol, double constrTol, VectorInteger constrIdxs)
{
    return (findZero(ptA, ptB, Nmax, lambdaTol, constrTol));
}
