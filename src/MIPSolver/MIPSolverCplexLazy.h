/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "MIPSolverCplex.h"
#include "MIPSolverCallbackBase.h"

#include <mutex>

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wignored-attributes"
#endif
#include "ilcplex/ilocplex.h"
#ifdef __GNUC__
#pragma GCC diagnostic warning "-Wignored-attributes"
#endif

namespace SHOT
{
class MIPSolverCplexLazy : public MIPSolverCplex
{
public:
    MIPSolverCplexLazy(EnvironmentPtr envPtr);
    ~MIPSolverCplexLazy() override;

    void checkParameters() override;

    void initializeSolverSettings() override;

    E_ProblemSolutionStatus solveProblem() override;

    int increaseSolutionLimit(int increment) override;
    void setSolutionLimit(long limit) override;
    int getSolutionLimit() override;

private:
protected:
};

class CplexCallback : public IloCplex::Callback::Function, public MIPSolverCallbackBase
{

private:
    std::mutex callbackMutex;
    /* Empty constructor is forbidden. */
    CplexCallback();

    /* Copy constructor is forbidden. */
    CplexCallback(const CplexCallback& tocopy);

    IloNumVarArray cplexVars;
    IloCplex cplexInst;

    void createHyperplane(Hyperplane hyperplane, const IloCplex::Callback::Context& context);
    void createIntegerCut(VectorInteger& binaryIndexesOnes, VectorInteger& binaryIndexesZeroes,
        const IloCplex::Callback::Context& context);

public:
    /* Constructor with data */
    CplexCallback(EnvironmentPtr envPtr, const IloNumVarArray& vars, const IloCplex& inst);

    void addLazyConstraint(std::vector<SolutionPoint> candidatePoints, const IloCplex::Callback::Context& context);

    // This is the function that we have to implement and that Cplex will call
    // during the solution process at the places that we asked for.
    void invoke(const IloCplex::Callback::Context& context) override;

    /// Destructor
    ~CplexCallback() override;
};
} // namespace SHOT