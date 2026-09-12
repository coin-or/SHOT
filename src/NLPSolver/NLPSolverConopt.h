/** The Supporting Hyperplane Optimization Toolkit (SHOT).
 * Licensed under the Eclipse Public License 2.0. See LICENSE.
 */
#pragma once
#include "NLPSolverBase.h"

namespace SHOT
{
class NLPSolverConopt : public NLPSolverBase
{
public:
    NLPSolverConopt(EnvironmentPtr envPtr, ProblemPtr source);
    void setStartingPoint(VectorInteger indexes, VectorDouble values) override;
    void clearStartingPoint() override;
    void fixVariables(VectorInteger indexes, VectorDouble values) override;
    void unfixVariables() override;
    void saveProblemToFile(std::string filename) override;
    void saveOptionsToFile(std::string filename) override;
    VectorDouble getSolution() override { return solution; }
    double getSolution(int i) override { return solution.at(i); }
    double getObjectiveValue() override;
    VectorDouble getVariableLowerBounds() override { return lowerBounds; }
    VectorDouble getVariableUpperBounds() override { return upperBounds; }
    void updateVariableLowerBound(int i, double bound) override;
    void updateVariableUpperBound(int i, double bound) override;
    std::string getSolverDescription() override;

protected:
    E_NLPSolutionStatus solveProblemInstance() override;

private:
    ProblemPtr sourceProblem;
    VectorDouble lowerBounds, upperBounds, lowerBeforeFix, upperBeforeFix, start, solution;
    bool invalidFix = false;
    bool warnedLicense = false;
    bool warnedIncompleteLicense = false;
};
} // namespace SHOT
