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
class TaskSelectPrimalCandidatesFromNLP;

// Solves an NLP problem for each combination of the values of the discrete variables, if there are not too many of
// them. See docs/ExhaustiveFixedIntegerSearchPlan.md
class TaskEnumerateFixedIntegerCombinations : public TaskBase
{
public:
    // isFallback marks the instance in the finalize sequence, which is only run if the objective gap could not be
    // closed. It skips the combinations that have been solved before
    TaskEnumerateFixedIntegerCombinations(EnvironmentPtr envPtr, bool isFallback);
    ~TaskEnumerateFixedIntegerCombinations() override;

    void run() override;
    std::string getType() override;

private:
    bool isFallback;

    bool isFallbackNeeded();

    // Sets the domains of the discrete variables, and returns false if the search cannot be performed
    bool initializeDomains();

    PrimalFixedNLPCandidate createCandidate(const VectorDouble& combination);

    // Returns false after the last combination
    bool selectNextCombination(VectorDouble& combination);

    // Whether the combination has been solved in the exhaustive search or used in the fixed-integer strategy
    bool hasCombinationBeenSolved(const PrimalFixedNLPCandidate& candidate);

    bool hasUnusedCombinations();

    void createNLPTasks();

    void enumerateCombinations();

    VectorInteger discreteVariableIndexes;
    VectorDouble domainLowerBounds;
    VectorDouble domainUpperBounds;
    int numberOfCombinations = 0;

    // The tasks solving the NLP problems, one for each problem in Primal.FixedInteger.SourceProblem
    std::vector<std::shared_ptr<TaskSelectPrimalCandidatesFromNLP>> NLPTasks;
};
} // namespace SHOT
