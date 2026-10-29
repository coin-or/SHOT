/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#include <memory>
#include <string>
#include <vector>

#include "../Structs.h"

namespace SHOT
{
class INLPSolver;

class TaskSelectPrimalCandidatesFromNLP : public TaskBase
{
public:
    // isFinalPolish marks the instance added to the finalize sequence
    TaskSelectPrimalCandidatesFromNLP(EnvironmentPtr envPtr, bool useReformulatedProblem, bool isFinalPolish = false);
    ~TaskSelectPrimalCandidatesFromNLP() override;
    void run() override;
    std::string getType() override;

    // Whether the NLP solver could be initialized
    bool hasNLPSolver() { return (NLPSolver != nullptr); }

    // Whether the NLP problems are created from the reformulated problem, which also depends on the NLP solver
    bool usesReformulatedProblem() { return (sourceIsReformulatedProblem); }

    // Solves the NLP problems with the discrete variables fixed to their values in the candidates, and returns the
    // status of the last one. In the exhaustive search (isEnumeration), no starting point is given for the continuous
    // variables, no cuts are added, and neither the call frequency nor the used candidates of the fixed-integer
    // strategy are updated
    E_NLPSolutionStatus solveFixedNLPCandidates(const std::vector<PrimalFixedNLPCandidate>& candidates,
        bool isEnumeration = false, double timeLimit = SHOT_DBL_MAX);

private:
    bool isFinalPolish;

    // Makes the calls of the fixed-integer NLP strategy more frequent after a successful NLP solve and less frequent
    // otherwise, within 0.1 and 10 times Primal.FixedInteger.Frequency.Iteration and .Time
    void updateFrequencies(bool isSuccessful);

    virtual bool solveFixedNLP();

    // Whether the objective function and the nonlinear constraints have finite values in the point
    bool hasFiniteFunctionValues(const VectorDouble& point);

    void createInfeasibilityCut(const VectorDouble point);

    // Adds hyperplanes in a feasible solution of the fixed-integer NLP problem for the convex nonlinear constraints
    // of the reformulated problem that are active in it, ranked by how much they cut away of the dual solution the
    // integer assignment comes from
    void createSupportingHyperplanes(
        const VectorDouble& point, const VectorDouble& dualSolution, double dualObjectiveValue);
    void createIntegerCut(VectorDouble point);

    std::shared_ptr<INLPSolver> NLPSolver;

    VectorInteger discreteVariableIndexes;
    std::vector<VectorDouble> testedPoints;
    VectorDouble fixPoint;

    VectorDouble originalLBs;
    VectorDouble originalUBs;

    VectorString variableNames;

    std::shared_ptr<TaskBase> taskSelectHPPts;

    ProblemPtr sourceProblem;
    bool sourceIsReformulatedProblem = false;
};
} // namespace SHOT