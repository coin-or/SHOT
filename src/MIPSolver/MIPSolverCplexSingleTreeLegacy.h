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

class HCallbackI : public IloCplex::HeuristicCallbackI, public MIPSolverCallbackBase
{
    IloNumVarArray cplexVars;

private:
public:
    IloCplex::CallbackI* duplicateCallback() const override;
    HCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2);
    void main() override;

    ~HCallbackI() override = default;
};

class InfoCallbackI : public IloCplex::MIPInfoCallbackI, public MIPSolverCallbackBase
{
private:
public:
    IloCplex::CallbackI* duplicateCallback() const override;
    InfoCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv);
    void main() override;

    ~InfoCallbackI() override = default;
};

class CtCallbackI : public IloCplex::LazyConstraintCallbackI, public MIPSolverCallbackBase
{
    IloNumVarArray cplexVars;

    bool createHyperplane(HyperplanePtr hyperplane);

    bool createIntegerCut(IntegerCut& integerCut);

public:
    IloCplex::CallbackI* duplicateCallback() const override;

    CtCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2);
    void main() override;

    ~CtCallbackI() override = default;
};

class MIPSolverCplexSingleTreeLegacy : public MIPSolverCplex
{
public:
    MIPSolverCplexSingleTreeLegacy(EnvironmentPtr envPtr);
    ~MIPSolverCplexSingleTreeLegacy() override
    {
        if(callbacksInitialized)
        {
            cplexInstance.remove(ctCallback);
            cplexInstance.remove(hCallback);
            cplexInstance.remove(infoCallback);
            delete ctCallback;
            delete hCallback;
            delete infoCallback;
            callbacksInitialized = false;
        }
    }

    void checkParameters() override;

    void initializeSolverSettings() override;

    E_ProblemSolutionStatus solveProblem() override;

    int increaseSolutionLimit(int increment) override;
    void setSolutionLimit(long limit) override;
    int getSolutionLimit() override;

    std::mutex callbackMutex2;

private:
    CtCallbackI* ctCallback;
    HCallbackI* hCallback;
    InfoCallbackI* infoCallback;
    bool callbacksInitialized = false;

protected:
};
} // namespace SHOT