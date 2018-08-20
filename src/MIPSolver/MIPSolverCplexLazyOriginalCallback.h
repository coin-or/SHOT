/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "IMIPSolver.h"
#include "MIPSolverBase.h"
#include "MIPSolverCallbackBase.h"

#include <functional>
#include <thread>
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
class MIPSolverCplexLazyOriginalCallback : public MIPSolverCplex
{
  public:
    MIPSolverCplexLazyOriginalCallback(EnvironmentPtr envPtr);
    virtual ~MIPSolverCplexLazyOriginalCallback();

    virtual void checkParameters();

    virtual void initializeSolverSettings();

    virtual E_ProblemSolutionStatus solveProblem();

    virtual int increaseSolutionLimit(int increment);
    virtual void setSolutionLimit(long limit);
    virtual int getSolutionLimit();

    std::mutex callbackMutex2;

  private:
    IloRangeArray cplexLazyConstrs;

  protected:
};

class HCallbackI : public IloCplex::HeuristicCallbackI, public MIPSolverCallbackBase
{
    IloNumVarArray cplexVars;

  private:
  public:
    IloCplex::CallbackI *duplicateCallback() const;
    HCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2);
    void main();

    virtual ~HCallbackI();
};

class InfoCallbackI : public IloCplex::MIPInfoCallbackI, public MIPSolverCallbackBase
{
    IloNumVarArray cplexVars;

  private:
  public:
    IloCplex::CallbackI *duplicateCallback() const;
    InfoCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2);
    void main();

    virtual ~InfoCallbackI();
};

class CtCallbackI : public IloCplex::LazyConstraintCallbackI, public MIPSolverCallbackBase
{
    IloNumVarArray cplexVars;

    MIPSolverCplexLazyOriginalCallback *cplexSolver;

    void createHyperplane(Hyperplane hyperplane);

    void createIntegerCut(VectorInteger binaryIndexes);

  public:
    IloCplex::CallbackI *duplicateCallback() const;

    CtCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2);
    void main();

    virtual ~CtCallbackI();
};
} // namespace SHOT