/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Shared.h"
#include "IMIPSolver.h"

namespace SHOT
{
class MIPSolverBase
{
  private:
    VectorInteger fixedVariableIndexes;
    std::vector<PairDouble> fixedVariableOriginalBounds;

  protected:
    std::vector<GeneratedHyperplane> generatedHyperplanes;

    int numberOfVariables = 0;
    int numberOfConstraints = 0;
    bool isMinimizationProblem;
    bool isProblemDiscrete = false;
    std::vector<E_VariableType> variableTypes;
    VectorDouble variableLowerBounds;
    VectorDouble variableUpperBounds;
    VectorString variableNames;

  public:
    ~MIPSolverBase();

    virtual void createHyperplane(Hyperplane hyperplane);
    virtual void createIntegerCut(VectorInteger binaryIndexes);

    virtual void createInteriorHyperplane(Hyperplane hyperplane);

    boost::optional<std::pair<std::vector<PairIndexValue>, double>> createHyperplaneTerms(Hyperplane hyperplane);

    virtual bool getDiscreteVariableStatus();
    virtual std::vector<SolutionPoint> getAllVariableSolutions();
    virtual int getNumberOfSolutions() = 0;
    virtual VectorDouble getVariableSolution(int i) = 0;
    virtual double getObjectiveValue(int i) = 0;
    virtual double getObjectiveValue();

    virtual void presolveAndUpdateBounds();
    virtual std::pair<VectorDouble, VectorDouble> presolveAndGetNewBounds() = 0;

    virtual std::vector<GeneratedHyperplane> *getGeneratedHyperplanes();

    virtual PairDouble getCurrentVariableBounds(int varIndex) = 0;

    virtual void fixVariable(int varIndex, double value) = 0;
    virtual void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues);
    virtual void unfixVariables();
    virtual void updateVariableBound(int varIndex, double lowerBound, double upperBound) = 0;

    virtual int addLinearConstraint(const std::vector<PairIndexValue> &elements, double constant) = 0;
    virtual int addLinearConstraint(const std::vector<PairIndexValue> &elements, double constant, bool isGreaterThan) = 0;

    virtual void activateDiscreteVariables(bool activate) = 0;

    virtual int getNumberOfExploredNodes() = 0;
    virtual int getNumberOfOpenNodes();

    bool discreteVariablesActivated;
    bool cachedSolutionHasChanged;

    bool isVariablesFixed;

    std::vector<SolutionPoint> lastSolutions;

    //OptProblem *originalProblem;
    //ProblemPtr sourceProblem;

    EnvironmentPtr env;
};
} // namespace SHOT