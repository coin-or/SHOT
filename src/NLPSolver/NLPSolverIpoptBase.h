/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "NLPSolverBase.h"
#include "../SharedOS.h"

namespace SHOT
{
class NLPSolverIpoptBase : virtual public INLPSolver
{
private:
protected:
    std::shared_ptr<OSInstance> osInstance;
    std::unique_ptr<OSOption> osOption;
    std::unique_ptr<IpoptSolver> IpoptNLPSolver;
    std::unique_ptr<OSoLWriter> osolwriter;

    VectorInteger fixedVariableIndexes;
    VectorDouble fixedVariableValues;

    VectorInteger startingPointVariableIndexes;
    VectorDouble startingPointVariableValues;

    virtual E_NLPSolutionStatus solveProblemInstance();

    void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues);
    void unfixVariables();

    void setIntegers(bool useDiscrete);

    virtual void setInitialSettings();
    virtual void setSolverSpecificInitialSettings() = 0;
    virtual void updateSettings();

    virtual VectorDouble getVariableLowerBounds();
    virtual VectorDouble getVariableUpperBounds();

    virtual void updateVariableLowerBound(int variableIndex, double bound);
    virtual void updateVariableUpperBound(int variableIndex, double bound);

    VectorDouble lowerBoundsBeforeFix;
    VectorDouble upperBoundsBeforeFix;

    std::vector<char> originalVariableType;

public:
    virtual ~NLPSolverIpoptBase(){};

    virtual void setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues);
    virtual void clearStartingPoint();

    virtual VectorDouble getSolution();
    virtual double getSolution(int i);
    virtual double getObjectiveValue();

    virtual void saveOptionsToFile(std::string fileName);
    virtual void saveProblemToFile(std::string fileName);
};
} // namespace SHOT