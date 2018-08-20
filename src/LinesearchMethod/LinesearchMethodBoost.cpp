/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "LinesearchMethodBoost.h"

using namespace SHOT;

VectorInteger activeConstraints;
double lastActiveConstraintUpdateValue;

Test::Test(EnvironmentPtr envPtr) : env(envPtr)
{
    nonlinearConstraints = env->model->originalProblem->getNonlinearConstraintIndexes();
}

Test::~Test()
{
    firstPt.clear();
    secondPt.clear();
}

void Test::determineActiveConstraints(double constrTol)
{
    valFirstPt = -OSDBL_MAX;
    valSecondPt = -OSDBL_MAX;

    clearActiveConstraints();

    for (auto I : nonlinearConstraints)
    {
        auto tmpValFirstPt = env->model->originalProblem->calculateConstraintFunctionValue(I, firstPt);
        auto tmpValSecondPt = env->model->originalProblem->calculateConstraintFunctionValue(I, secondPt);

        if ((tmpValFirstPt > constrTol && tmpValSecondPt <= 0) || (tmpValFirstPt <= 0 && tmpValSecondPt > constrTol))
        {
            addActiveConstraint(I);
        }

        // For reuse of the function value
        if (tmpValFirstPt > valFirstPt)
            valFirstPt = tmpValFirstPt;
        if (tmpValSecondPt > valSecondPt)
            valSecondPt = tmpValSecondPt;
    }

    lastActiveConstraintUpdateValue = OSDBL_MAX;
}

void Test::addActiveConstraint(int constrIdx)
{
    activeConstraints.push_back(constrIdx);
}

void Test::clearActiveConstraints()
{
    activeConstraints.clear();
}

void Test::setActiveConstraints(VectorInteger constrIdxs)
{
    activeConstraints = constrIdxs;
}

VectorInteger Test::getActiveConstraints()
{
    return (activeConstraints);
}

double Test::operator()(const double x)
{
    int length = firstPt.size();
    VectorDouble ptNew(length);

    for (int i = 0; i < length; i++)
    {
        ptNew.at(i) = x * firstPt.at(i) + (1 - x) * secondPt.at(i);
    }

    auto tmpActiveConstraints = getActiveConstraints();
    auto mostDevConstr = env->model->originalProblem->getMostDeviatingConstraint(ptNew, tmpActiveConstraints);

    double validNewPt = mostDevConstr.first.value;

    if (validNewPt > 0 && validNewPt <= lastActiveConstraintUpdateValue && mostDevConstr.second.size() < tmpActiveConstraints.size())
    {
        setActiveConstraints(mostDevConstr.second);
        lastActiveConstraintUpdateValue = validNewPt;
    }

    return (validNewPt);
}

LinesearchMethodBoost::LinesearchMethodBoost(EnvironmentPtr envPtr) : env(envPtr)
{
    test = new Test(env);
}

LinesearchMethodBoost::~LinesearchMethodBoost()
{
    activeConstraints.clear();
    delete test;
}

std::pair<VectorDouble, VectorDouble> LinesearchMethodBoost::findZero(VectorDouble ptA,
                                                                      VectorDouble ptB, int Nmax, double lambdaTol, double constrTol)
{
    VectorInteger tmpVector;
    return (findZero(ptA, ptB, Nmax, lambdaTol, constrTol, tmpVector));
}

std::pair<VectorDouble, VectorDouble> LinesearchMethodBoost::findZero(VectorDouble ptA,
                                                                      VectorDouble ptB, int Nmax, double lambdaTol, double constrTol, VectorInteger constrIdxs)
{
    if (ptA.size() != ptB.size())
    {
        env->output->outputError(
            "     Linesearch error: sizes of points vary: " + std::to_string(ptA.size()) + " != " + std::to_string(ptB.size()));
    }

    int length = ptA.size();
    VectorDouble ptNew(length);
    VectorDouble ptNew2(length);

    typedef PairDouble Result;
    boost::uintmax_t max_iter = Nmax;

    test->firstPt = ptA;
    test->secondPt = ptB;

    if (constrIdxs.size() == 0)
    {
        test->determineActiveConstraints(constrTol);
    }
    else
    {
        test->setActiveConstraints(constrIdxs);
        test->valFirstPt = env->model->originalProblem->getMostDeviatingConstraint(ptA).value;
        test->valSecondPt = env->model->originalProblem->getMostDeviatingConstraint(ptB).value;
    }

    if (test->getActiveConstraints().size() == 0) // All constraints are fulfilled.
    {
        if (test->valFirstPt > test->valSecondPt)
        {
            std::pair<VectorDouble, VectorDouble> tmpPair(ptB, ptA);

            return (tmpPair);
        }

        std::pair<VectorDouble, VectorDouble> tmpPair(ptA, ptB);

        return (tmpPair);
    }

    int tempFEvals = env->solutionStatistics.numberOfFunctionEvalutions;

    Result r1;

    if (static_cast<ES_RootsearchMethod>(env->settings->getIntSetting("Rootsearch.Method", "Subsolver")) == ES_RootsearchMethod::BoostTOMS748)
    {
        r1 = boost::math::tools::toms748_solve(*test, 0.0, 1.0, TerminationCondition(lambdaTol), max_iter);
    }
    else
    {
        r1 = boost::math::tools::bisect(*test, 0.0, 1.0, TerminationCondition(lambdaTol), max_iter);
    }

    int resFVals = env->solutionStatistics.numberOfFunctionEvalutions - tempFEvals;
    if (max_iter == Nmax)
    {
        env->output->outputWarning(
            "     Warning, number of line search iterations " + std::to_string(max_iter) + " reached!");
    }
    else
    {
        env->output->outputInfo(
            "     Line search iterations: " + std::to_string(max_iter) + ". Function evaluations: " + std::to_string(resFVals));
    }

    for (int i = 0; i < length; i++)
    {
        ptNew.at(i) = r1.first * ptA.at(i) + (1 - r1.first) * ptB.at(i);
        ptNew2.at(i) = r1.second * ptA.at(i) + (1 - r1.second) * ptB.at(i);
    }

    auto validNewPt = env->model->originalProblem->isConstraintsFulfilledInPoint(ptNew);

    if (!validNewPt) // ptNew Outside feasible region
    {
        env->process->addPrimalSolutionCandidate(ptNew2, E_PrimalSolutionSource::Linesearch,
                                                 env->process->getCurrentIteration()->iterationNumber);

        std::pair<VectorDouble, VectorDouble> tmpPair(ptNew2, ptNew);
        return (tmpPair);
    }
    else
    {
        env->process->addPrimalSolutionCandidate(ptNew, E_PrimalSolutionSource::Linesearch,
                                                 env->process->getCurrentIteration()->iterationNumber);

        std::pair<VectorDouble, VectorDouble> tmpPair(ptNew, ptNew2);
        return (tmpPair);
    }
}
