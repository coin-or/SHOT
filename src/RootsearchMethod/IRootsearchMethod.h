/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Environment.h"
#include "../Structs.h"

#include "../Model/Constraints.h"
#include "../Model/ObjectiveFunction.h"

namespace SHOT
{

class IRootsearchMethod
{
public:
    IRootsearchMethod() = default;
    IRootsearchMethod(EnvironmentPtr envPtr [[maybe_unused]]) {};
    virtual ~IRootsearchMethod() = default;

    virtual std::pair<VectorDouble, VectorDouble> findZero(const VectorDouble& ptA, const VectorDouble& ptB, int Nmax,
        double lambdaTol, double constrTol, const std::vector<NumericConstraint*> constraints, bool addPrimalCandidate)
        = 0;

    virtual std::pair<VectorDouble, VectorDouble> findZero(const VectorDouble& ptA, const VectorDouble& ptB, int Nmax,
        double lambdaTol, double constrTol, const NonlinearConstraints constraints, bool addPrimalCandidate)
        = 0;

    virtual std::pair<double, double> findZero(const VectorDouble& pt, double objectiveLB, double objectiveUB, int Nmax,
        double lambdaTol, double constrTol, ObjectiveFunctionPtr objectiveFunction)
        = 0;

protected:
    EnvironmentPtr env;
};
} // namespace SHOT