/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "MIPSolverBase.h"
#include "MIPSolverGurobi.h"
#include "MIPSolverCallbackBase.h"

namespace SHOT
{
class MIPSolverGurobiLazy : public MIPSolverGurobi
{
public:
    MIPSolverGurobiLazy(EnvironmentPtr envPtr);
    virtual ~MIPSolverGurobiLazy();

    virtual void checkParameters();

    virtual void initializeSolverSettings();

    virtual int increaseSolutionLimit(int increment);
    virtual void setSolutionLimit(long limit);
    virtual int getSolutionLimit();

    E_ProblemSolutionStatus solveProblem();

private:
};

class GurobiCallback : public GRBCallback, public MIPSolverCallbackBase
{
public:
    GRBVar* vars;
    GurobiCallback(GRBVar* xvars, EnvironmentPtr envPtr);

protected:
    void callback();

private:
    int numVar = 0;
    int lastExploredNodes = 0;
    int lastOpenNodes = 0;

    void createHyperplane(Hyperplane hyperplane);
    void createIntegerCut(VectorInteger &binaryIndexes);

    void addLazyConstraint(std::vector<SolutionPoint> candidatePoints);
};
} // namespace SHOT