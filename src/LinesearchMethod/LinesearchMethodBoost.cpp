/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "LinesearchMethodBoost.h"

namespace SHOT
{
//VectorInteger activeConstraints;
std::vector<NonlinearConstraint *> activeConstraints;
double lastActiveConstraintUpdateValue;

Test::Test(EnvironmentPtr envPtr) : env(envPtr)
{
}

Test::~Test()
{
    firstPt.clear();
    secondPt.clear();
}

void Test::addActiveConstraint(NonlinearConstraint *constraint)
{
    activeConstraints.push_back(constraint);
}

void Test::clearActiveConstraints()
{
    activeConstraints.clear();
}

void Test::setActiveConstraints(const std::vector<NonlinearConstraint *> &constraints)
{
    for (auto C : constraints)
        addActiveConstraint(C);
}

std::vector<NonlinearConstraint *> Test::getActiveConstraints()
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

    double calculatedValue = NAN;

    auto currentConstraints = getActiveConstraints();

    std::vector<NonlinearConstraint *> activeConstraints;

    if (auto sharedProblem = problem.lock())
    {
        auto constraintValue = sharedProblem->getMaxNumericConstraintValue(ptNew, currentConstraints, activeConstraints);
        calculatedValue = constraintValue.normalizedValue;

        if (!constraintValue.isFulfilled && calculatedValue <= lastActiveConstraintUpdateValue && activeConstraints.size() < currentConstraints.size())
        {
            setActiveConstraints(activeConstraints);
            lastActiveConstraintUpdateValue = calculatedValue;
        }
    }

    return (calculatedValue);
}

TestObjective::TestObjective(EnvironmentPtr envPtr) : env(envPtr)
{
}

TestObjective::~TestObjective()
{
}

double TestObjective::operator()(const double x)
{
    // Change the value of the auxilliary objective function variable
    double ptNew = x * firstPt + (1 - x) * secondPt;

    double calculatedValue = cachedObjectiveValue - ptNew;

    //std::cout << firstPt << " < " << ptNew << " < " << secondPt << ": " << calculatedValue << std::endl;

    return (calculatedValue);
}

LinesearchMethodBoost::LinesearchMethodBoost(EnvironmentPtr envPtr) : env(envPtr)
{
    test = new Test(env);
    testObjective = new TestObjective(env);
}

LinesearchMethodBoost::~LinesearchMethodBoost()
{
    activeConstraints.clear();
    delete test;
    delete testObjective;
}

std::pair<VectorDouble, VectorDouble> LinesearchMethodBoost::findZero(const VectorDouble &ptA, const VectorDouble &ptB,
                                                                      int Nmax, double lambdaTol, double constrTol,
                                                                      const NonlinearConstraints &constraints)
{

    if (ptA.size() != ptB.size())
    {
        env->output->outputError(
            "     Line search error: sizes of points vary: " + std::to_string(ptA.size()) + " != " + std::to_string(ptB.size()));
    }

    if (constraints.size() == 0)
    {
        env->output->outputError("     No constraints selected for line search");
    }

    test->problem = constraints[0]->ownerProblem;

    int length = ptA.size();
    VectorDouble ptNew(length);
    VectorDouble ptNew2(length);

    typedef PairDouble Result;
    boost::uintmax_t max_iter = Nmax;

    test->firstPt = ptA;
    test->secondPt = ptB;

    std::vector<NonlinearConstraint *> firstActiveConstraints;
    std::vector<NonlinearConstraint *> secondActiveConstraints;

    if (auto sharedProblem = test->problem.lock())
    {
        test->valFirstPt = sharedProblem->getMaxNumericConstraintValue(ptA, constraints, firstActiveConstraints).normalizedValue;
        test->valSecondPt = sharedProblem->getMaxNumericConstraintValue(ptB, constraints, secondActiveConstraints).normalizedValue;

        if (test->valFirstPt > 0)
            test->setActiveConstraints(firstActiveConstraints);
        else
            test->setActiveConstraints(secondActiveConstraints);
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

    auto validNewPt = env->problem->areNonlinearConstraintsFulfilled(ptNew, 0);

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

std::pair<double, double> LinesearchMethodBoost::findZero(const VectorDouble &pt, double objectiveLB, double objectiveUB,
                                                          int Nmax, double lambdaTol, double constrTol,
                                                          const NonlinearObjectiveFunctionPtr &objectiveFunction)
{
    testObjective->solutionPoint = pt;
    testObjective->firstPt = objectiveLB;
    testObjective->secondPt = objectiveUB;

    if (auto sharedProblem = objectiveFunction->ownerProblem.lock())
    {
        testObjective->cachedObjectiveValue = sharedProblem->objectiveFunction->calculateValue(pt);
    }

    typedef PairDouble Result;
    boost::uintmax_t max_iter = Nmax;

    int tempFEvals = env->solutionStatistics.numberOfFunctionEvalutions;

    Result r1;

    if (static_cast<ES_RootsearchMethod>(env->settings->getIntSetting("Rootsearch.Method", "Subsolver")) == ES_RootsearchMethod::BoostTOMS748)
    {
        r1 = boost::math::tools::toms748_solve(*testObjective, 0.0, 1.0, TerminationCondition(lambdaTol), max_iter);
    }
    else
    {
        r1 = boost::math::tools::bisect(*testObjective, 0.0, 1.0, TerminationCondition(lambdaTol), max_iter);
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

    double ptNew = r1.first * objectiveLB + (1 - r1.first) * objectiveUB;
    double ptNew2 = r1.second * objectiveLB + (1 - r1.second) * objectiveUB;

    if (ptNew2 < ptNew)
    {
        // TODO: check that this is not needed
        /*env->process->addPrimalSolutionCandidate(ptNew2, E_PrimalSolutionSource::Linesearch,
                                                 env->process->getCurrentIteration()->iterationNumber);*/

        std::pair<double, double> tmpPair(ptNew2, ptNew);
        return (tmpPair);
    }
    else
    {
        // TODO: check that this is not needed
        /*env->process->addPrimalSolutionCandidate(ptNew, E_PrimalSolutionSource::Linesearch,
                                                 env->process->getCurrentIteration()->iterationNumber);*/

        std::pair<double, double> tmpPair(ptNew, ptNew2);
        return (tmpPair);
    }
}
} // namespace SHOT
