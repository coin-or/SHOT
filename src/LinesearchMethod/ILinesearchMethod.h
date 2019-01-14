/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Shared.h"

namespace SHOT
{
class ILinesearchMethod
{
public:
    ILinesearchMethod(){};
    ILinesearchMethod(EnvironmentPtr envPtr){};
    virtual ~ILinesearchMethod(){};

    // virtual std::pair<VectorDouble, VectorDouble> findZero(VectorDouble ptA, VectorDouble ptB,
    //                                                       int Nmax, double lambdaTol, double constrTol) = 0;

    virtual std::pair<VectorDouble, VectorDouble> findZero(const VectorDouble& ptA, const VectorDouble& ptB, int Nmax,
        double lambdaTol, double constrTol, const std::vector<NumericConstraint*> constraints)
        = 0;

    virtual std::pair<VectorDouble, VectorDouble> findZero(const VectorDouble& ptA, const VectorDouble& ptB, int Nmax,
        double lambdaTol, double constrTol, const NonlinearConstraints constraints)
        = 0;

    virtual std::pair<double, double> findZero(const VectorDouble& pt, double objectiveLB, double objectiveUB, int Nmax,
        double lambdaTol, double constrTol, const NonlinearObjectiveFunction* objectiveFunction)
        = 0;

protected:
    EnvironmentPtr env;
};
} // namespace SHOT