/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "IRootsearchMethod.h"
#include "../Environment.h"

#include <algorithm>
#include <cmath>

namespace SHOT
{
class Test
{
private:
    EnvironmentPtr env;

public:
    Problem* problem;

    VectorDouble firstPt;
    VectorDouble secondPt;

    double valFirstPt;
    double valSecondPt;

    Test(EnvironmentPtr envPtr);
    ~Test();

    void setActiveConstraints(const std::vector<NumericConstraint*>& constraints);
    std::vector<NumericConstraint*> getActiveConstraints();
    void clearActiveConstraints();
    void addActiveConstraint(NumericConstraint* constraint);

    double operator()(const double x);
};

class TestObjective
{
private:
    EnvironmentPtr env;

public:
    VectorDouble solutionPoint;
    double cachedObjectiveValue;

    double firstPt;
    double secondPt;

    TestObjective(EnvironmentPtr envPtr);
    ~TestObjective();

    double operator()(const double x);
};

class TerminationCondition
{
private:
    double tol;

public:
    TerminationCondition(double tolerance) { tol = tolerance; }

    // The interval also cannot be made smaller when there is no double between its ends, e.g., for a small tolerance
    // and lambda close to 1, where doubles are further apart than the tolerance
    bool operator()(double min, double max)
    {
        double lower = std::min(min, max);
        double upper = std::max(min, max);

        return (upper - lower <= tol || std::nextafter(lower, upper) >= upper);
    }
};

class RootsearchMethodBoost : public IRootsearchMethod
{
public:
    RootsearchMethodBoost(EnvironmentPtr envPtr);
    ~RootsearchMethodBoost() override;

    std::pair<VectorDouble, VectorDouble> findZero(const VectorDouble& ptA, const VectorDouble& ptB, int Nmax,
        double lambdaTol, const NonlinearConstraints constraints, bool addPrimalCandidate) override;

    std::pair<VectorDouble, VectorDouble> findZero(const VectorDouble& ptA, const VectorDouble& ptB, int Nmax,
        double lambdaTol, const std::vector<NumericConstraint*> constraints, bool addPrimalCandidate) override;

    std::pair<double, double> findZero(const VectorDouble& pt, double objectiveLB, double objectiveUB, int Nmax,
        double lambdaTol, ObjectiveFunctionPtr objectiveFunction) override;

private:
    std::unique_ptr<Test> test;
    std::unique_ptr<TestObjective> testObjective;
    EnvironmentPtr env;
};
} // namespace SHOT