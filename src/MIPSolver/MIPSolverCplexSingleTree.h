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

#include <atomic>
#include <cmath>
#include <mutex>
#include <optional>
#include <set>

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wignored-attributes"
#endif
#include "ilcplex/ilocplex.h"
#ifdef __GNUC__
#pragma GCC diagnostic warning "-Wignored-attributes"
#endif

namespace SHOT
{
class MIPSolverCplexSingleTree : public MIPSolverCplex
{
public:
    MIPSolverCplexSingleTree(EnvironmentPtr envPtr);
    ~MIPSolverCplexSingleTree() override;

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
    CplexCallback() = delete;

    /* Copy constructor is forbidden. */
    CplexCallback(const CplexCallback& tocopy) = delete;

    IloNumVarArray cplexVars;

    // The tolerance with which Cplex considers a constraint fulfilled
    double feasibilityTolerance;

    // The last global dual bound from Cplex, if it has not been passed to SHOT yet
    std::atomic<double> globalDualBound = NAN;

    // A constraint to reject the current candidate with
    struct RejectionConstraint
    {
        std::vector<PairIndexValue> terms;
        double lowerBound;
        double upperBound;
        bool isLocal;
    };

    // The constraints to reject the current candidate with, which are given to Cplex at once
    std::vector<RejectionConstraint> rejectionConstraints;

    // An external dual bound not yet passed to Cplex, since no candidate has violated it
    std::optional<double> pendingExternalDualBound;

    // A hyperplane given to Cplex when rejecting a candidate, as terms and right-hand side
    struct RejectionCut
    {
        std::vector<PairIndexValue> terms;
        double rightHandSide;
        bool isAddedAsUserCut = false;
    };

    std::vector<RejectionCut> rejectionCuts;

    // The indexes in rejectionCuts of the hyperplanes to add as user cuts in the next relaxation context
    std::vector<size_t> pendingUserCuts;

    // The candidate points seen, to count those Cplex reports again
    std::set<PairDouble> candidatePointHashes;

    struct Statistics
    {
        int candidateCalls = 0;
        int candidateRays = 0;
        int repeatedCandidatePoints = 0;
        int rejectedCandidates = 0;
        int submittedHyperplanes = 0;
        int relaxationCalls = 0;
        int addedUserCuts = 0;
        int candidatesViolatingRejectionCuts = 0;
        int fixedNLPStrategyCalls = 0;
        int fixedNLPRepeatedCandidates = 0;
    } statistics;

    // Aborts the solve if a termination criterion is met
    bool checkTermination(const IloCplex::Callback::Context& context);

    void updateDualBound();

    // These return false if the solve has been aborted
    bool handleRelaxation(const IloCplex::Callback::Context& context);
    bool handleCandidate(const IloCplex::Callback::Context& context);

    // The best primal solution in the variables of Cplex and its objective value, if it has not been passed to Cplex
    std::optional<std::pair<VectorDouble, double>> getPrimalSolutionToPost();
    void postPrimalSolution(
        const IloCplex::Callback::Context& context, const VectorDouble& solution, double objectiveValue);

    void addPendingUserCuts(const IloCplex::Callback::Context& context);

    // Rejects the current candidate with the constraints collected for it
    void rejectCandidate(const IloCplex::Callback::Context& context);

    bool createHyperplane(HyperplanePtr hyperplane, bool isNodeCandidate);
    bool createIntegerCut(IntegerCut& integerCut);

    void addLazyConstraint(std::vector<SolutionPoint> candidatePoints, const IloCplex::Callback::Context& context);

    void addExternalDualBoundLazyConstraint(const IloCplex::Callback::Context& context);

public:
    /* Constructor with data */
    CplexCallback(EnvironmentPtr envPtr, const IloNumVarArray& vars, double feasibilityTolerance);

    // The contexts the callback needs to be registered for
    CPXLONG getContextMask();

    void outputStatistics();

    // This is the function that we have to implement and that Cplex will call
    // during the solution process at the places that we asked for.
    void invoke(const IloCplex::Callback::Context& context) override;

    /// Destructor
    ~CplexCallback() override;
};
} // namespace SHOT