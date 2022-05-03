/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "RootsearchMethodBoost.h"
#include "../Output.h"
#include "../Settings.h"
#include "../Model/Problem.h"
#include "../Results.h"
#include "../PrimalSolver.h"
#include "../Iteration.h"

#include "boost/math/tools/roots.hpp"
#include "boost/cstdint.hpp"

namespace SHOT
{
std::vector<NumericConstraint*> activeConstraints;
double lastActiveConstraintUpdateValue;

Test::Test(EnvironmentPtr envPtr) : env(envPtr) {}

Test::~Test()
{
    firstPt.clear();
    secondPt.clear();
}

void Test::addActiveConstraint(NumericConstraint* constraint) { activeConstraints.push_back(constraint); }

void Test::clearActiveConstraints() { activeConstraints.clear(); }

void Test::setActiveConstraints(const std::vector<NumericConstraint*>& constraints)
{
    clearActiveConstraints();

    for(auto& C : constraints)
        addActiveConstraint(C);
}

std::vector<NumericConstraint*> Test::getActiveConstraints() { return (activeConstraints); }

double Test::operator()(const double x)
{
    auto length = firstPt.size();
    VectorDouble ptNew(length);

    for(size_t i = 0; i < length; i++)
    {
        ptNew.at(i) = x * firstPt.at(i) + (1 - x) * secondPt.at(i);
    }

    auto currentConstraints = getActiveConstraints();

    std::vector<NumericConstraint*> activeConstraints;

    auto constraintValue = problem->getMaxNumericConstraintValue(ptNew, currentConstraints, activeConstraints);
    double calculatedValue = constraintValue.normalizedValue;

    if(!constraintValue.isFulfilled && calculatedValue <= lastActiveConstraintUpdateValue
        && activeConstraints.size() < currentConstraints.size())
    {
        setActiveConstraints(activeConstraints);
        lastActiveConstraintUpdateValue = calculatedValue;
    }

    return (calculatedValue);
}

TestObjective::TestObjective(EnvironmentPtr envPtr) : env(envPtr) {}

TestObjective::~TestObjective() = default;

double TestObjective::operator()(const double x)
{
    // Change the value of the auxiliary objective function variable
    double ptNew = x * firstPt + (1 - x) * secondPt;

    double calculatedValue = cachedObjectiveValue - ptNew;

    return (calculatedValue);
}

RootsearchMethodBoost::RootsearchMethodBoost(EnvironmentPtr envPtr) : env(envPtr)
{
    test = std::make_unique<Test>(env);
    testObjective = std::make_unique<TestObjective>(env);
}

RootsearchMethodBoost::~RootsearchMethodBoost() { activeConstraints.clear(); }

std::pair<VectorDouble, VectorDouble> RootsearchMethodBoost::findZero(const VectorDouble& ptA, const VectorDouble& ptB,
    int Nmax, double lambdaTol, double constrTol, const NonlinearConstraints constraints,
    bool addPrimalCandidate = true)
{
    std::vector<NumericConstraint*> tmpConstraints;
    tmpConstraints.reserve(size(constraints));

    for(auto& C : env->reformulatedProblem->nonlinearConstraints)
    {
        tmpConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(C).get());
    }

    return (RootsearchMethodBoost::findZero(ptA, ptB, Nmax, lambdaTol, constrTol, tmpConstraints, addPrimalCandidate));
}

std::pair<VectorDouble, VectorDouble> RootsearchMethodBoost::findZero(const VectorDouble& ptA, const VectorDouble& ptB,
    int Nmax, double lambdaTol, [[maybe_unused]] double constrTol, const std::vector<NumericConstraint*> constraints,
    bool addPrimalCandidate = true)
{
    if(ptA.size() != ptB.size())
    {
        env->output->outputError("        Root search error: sizes of points vary: " + std::to_string(ptA.size())
            + " != " + std::to_string(ptB.size()));
    }

    if(constraints.size() == 0)
    {
        env->output->outputError("        No constraints selected for root search");
    }

    if(auto sharedProblem = constraints[0]->ownerProblem.lock())
    {
        test->problem = sharedProblem.get();
    }

    auto length = ptA.size();
    VectorDouble ptNew(length);
    VectorDouble ptNew2(length);

    boost::uintmax_t max_iter = Nmax;

    test->firstPt = ptA;
    test->secondPt = ptB;

    std::vector<NumericConstraint*> firstActiveConstraints;
    std::vector<NumericConstraint*> secondActiveConstraints;

    test->valFirstPt
        = test->problem->getMaxNumericConstraintValue(ptA, constraints, firstActiveConstraints).normalizedValue;
    test->valSecondPt
        = test->problem->getMaxNumericConstraintValue(ptB, constraints, secondActiveConstraints).normalizedValue;

    if(test->valFirstPt > 0)
        test->setActiveConstraints(firstActiveConstraints);
    else
        test->setActiveConstraints(secondActiveConstraints);

    if(test->getActiveConstraints().size() == 0) // All constraints are fulfilled.
    {
        if(test->valFirstPt > test->valSecondPt)
        {
            std::pair<VectorDouble, VectorDouble> tmpPair(ptB, ptA);

            return (tmpPair);
        }

        std::pair<VectorDouble, VectorDouble> tmpPair(ptA, ptB);

        return (tmpPair);
    }

    int tempFEvals = env->solutionStatistics.numberOfFunctionEvalutions;

    PairDouble r1;

    if(static_cast<ES_RootsearchMethod>(env->settings->getSetting<int>("Rootsearch.Method", "Subsolver"))
        == ES_RootsearchMethod::BoostTOMS748)
    {
        r1 = boost::math::tools::toms748_solve(*test, 0.0, 1.0, TerminationCondition(lambdaTol), max_iter);
    }
    else
    {
        r1 = boost::math::tools::bisect(*test, 0.0, 1.0, TerminationCondition(lambdaTol), max_iter);
    }

    int resFVals = env->solutionStatistics.numberOfFunctionEvalutions - tempFEvals;
    if((int)max_iter == Nmax)
    {
        env->output->outputDebug(
            "        Warning, number of line search iterations " + std::to_string(max_iter) + " reached!");
    }
    else
    {
        env->output->outputTrace("        Line search iterations: " + std::to_string(max_iter)
            + ". Function evaluations: " + std::to_string(resFVals));
    }

    for(size_t i = 0; i < length; i++)
    {
        ptNew.at(i) = r1.first * ptA.at(i) + (1 - r1.first) * ptB.at(i);
        ptNew2.at(i) = r1.second * ptA.at(i) + (1 - r1.second) * ptB.at(i);
    }

    auto validNewPt = test->problem->areNonlinearConstraintsFulfilled(ptNew, 0);

    if(!validNewPt) // ptNew Outside feasible region
    {
        if(addPrimalCandidate)
        {
            env->primalSolver->addPrimalSolutionCandidate(
                ptNew2, E_PrimalSolutionSource::Rootsearch, env->results->getCurrentIteration()->iterationNumber);
        }

        std::pair<VectorDouble, VectorDouble> tmpPair(ptNew2, ptNew);
        return (tmpPair);
    }
    else
    {
        if(addPrimalCandidate)
        {
            env->primalSolver->addPrimalSolutionCandidate(
                ptNew, E_PrimalSolutionSource::Rootsearch, env->results->getCurrentIteration()->iterationNumber);
        }

        std::pair<VectorDouble, VectorDouble> tmpPair(ptNew, ptNew2);
        return (tmpPair);
    }
}

std::pair<double, double> RootsearchMethodBoost::findZero(const VectorDouble& pt, double objectiveLB,
    double objectiveUB, int Nmax, double lambdaTol, [[maybe_unused]] double constrTol,
    ObjectiveFunctionPtr objectiveFunction)
{
    testObjective->solutionPoint = pt;
    testObjective->firstPt = objectiveLB;
    testObjective->secondPt = objectiveUB;

    testObjective->cachedObjectiveValue = objectiveFunction->calculateValue(pt);

    boost::uintmax_t max_iter = Nmax;

    int tempFEvals = env->solutionStatistics.numberOfFunctionEvalutions;

    PairDouble r1;

    if(static_cast<ES_RootsearchMethod>(env->settings->getSetting<int>("Rootsearch.Method", "Subsolver"))
        == ES_RootsearchMethod::BoostTOMS748)
    {
        r1 = boost::math::tools::toms748_solve(*testObjective, 0.0, 1.0, TerminationCondition(lambdaTol), max_iter);
    }
    else
    {
        r1 = boost::math::tools::bisect(*testObjective, 0.0, 1.0, TerminationCondition(lambdaTol), max_iter);
    }

    int resFVals = env->solutionStatistics.numberOfFunctionEvalutions - tempFEvals;
    if((int)max_iter == Nmax)
    {
        env->output->outputDebug(
            "        Warning, number of line search iterations " + std::to_string(max_iter) + " reached!");
    }
    else
    {
        env->output->outputTrace("        Line search iterations: " + std::to_string(max_iter)
            + ". Function evaluations: " + std::to_string(resFVals));
    }

    double ptNew = r1.first * objectiveLB + (1 - r1.first) * objectiveUB;
    double ptNew2 = r1.second * objectiveLB + (1 - r1.second) * objectiveUB;

    if(ptNew2 < ptNew)
    {
        std::pair<double, double> tmpPair(ptNew2, ptNew);
        return (tmpPair);
    }
    else
    {
        std::pair<double, double> tmpPair(ptNew, ptNew2);
        return (tmpPair);
    }
}
} // namespace SHOT
