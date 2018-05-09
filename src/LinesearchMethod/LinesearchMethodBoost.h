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

#include "boost/math/tools/roots.hpp"

class Test
{
  private:
    EnvironmentPtr env;
    std::vector<int> nonlinearConstraints;

  public:
    std::vector<double> firstPt;
    std::vector<double> secondPt;

    double valFirstPt;
    double valSecondPt;

    Test(EnvironmentPtr envPtr);
    ~Test();
    void determineActiveConstraints(double constrTol);
    void setActiveConstraints(std::vector<int> constrIdxs);
    std::vector<int> getActiveConstraints();
    void clearActiveConstraints();
    void addActiveConstraint(int constrIdx);

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

    virtual std::pair<std::vector<double>, std::vector<double>> findZero(std::vector<double> ptA,
                                                                         std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol);

    virtual std::pair<std::vector<double>, std::vector<double>> findZero(std::vector<double> ptA,
                                                                         std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol, std::vector<int> constrIdxs);

  private:
    Test *test;
    EnvironmentPtr env;
};
