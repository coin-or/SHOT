/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "ILinesearchMethod.h"
#include "SHOTSettings.h"
#include "../Model.h"
#include "../OptProblems/OptProblemOriginal.h"
#include "../ProcessInfo.h"
#include "../Environment.h"
#include "../Model/Problem.h"
#include "../Model/Constraints.h"

#include "boost/math/tools/roots.hpp"
#include <memory>

namespace SHOT
{
class Test
{
  private:
    EnvironmentPtr env;

  public:
    std::weak_ptr<Problem> problem;

    VectorDouble firstPt;
    VectorDouble secondPt;

    double valFirstPt;
    double valSecondPt;

    Test(EnvironmentPtr envPtr);
    ~Test();

    void setActiveConstraints(const std::vector<NonlinearConstraint *> &constraints);
    std::vector<NonlinearConstraint *> getActiveConstraints();
    void clearActiveConstraints();
    void addActiveConstraint(NonlinearConstraint *constraint);

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
    TerminationCondition(double tolerance)
    {
        tol = tolerance;
    }

    bool operator()(double min, double max)
    {
        return (abs(min - max) <= tol);
    }
};

class LinesearchMethodBoost : public ILinesearchMethod
{
  public:
    LinesearchMethodBoost(EnvironmentPtr envPtr);
    virtual ~LinesearchMethodBoost();

    virtual std::pair<VectorDouble, VectorDouble> findZero(const VectorDouble &ptA, const VectorDouble &ptB,
                                                           int Nmax, double lambdaTol, double constrTol,
                                                           const NonlinearConstraints &constraints);

    virtual std::pair<double, double> findZero(const VectorDouble &pt, double objectiveLB, double objectiveUB,
                                               int Nmax, double lambdaTol, double constrTol,
                                               const NonlinearObjectiveFunctionPtr &objectiveFunction);

  private:
    Test *test;
    TestObjective *testObjective;
    EnvironmentPtr env;
};
} // namespace SHOT