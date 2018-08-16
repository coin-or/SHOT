/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "NLPSolverBase.h"

#include "gmomcc.h"
#include "gevmcc.h"

class NLPSolverGAMS : public NLPSolverBase
{
  private:
    gmoHandle_t gmo;
    gevHandle_t gev;

    char nlpsolver[GMS_SSSIZE];
    char nlpsolveropt[GMS_SSSIZE];
    double timelimit;
    int iterlimit;
    bool showlog;

  public:
    NLPSolverGAMS(EnvironmentPtr envPtr);

    virtual ~NLPSolverGAMS();

    void setStartingPoint(std::vector<int> variableIndexes, DoubleVector variableValues);
    void clearStartingPoint();

    void fixVariables(std::vector<int> variableIndexes, DoubleVector variableValues);

    void unfixVariables();

    void saveOptionsToFile(std::string fileName);

    DoubleVector getSolution();
    double getSolution(int i);
    double getObjectiveValue();

    bool isObjectiveFunctionNonlinear();
    int getObjectiveFunctionVariableIndex();

  protected:
    E_NLPSolutionStatus solveProblemInstance();
    bool createProblemInstance(OSInstance *origInstance);

    DoubleVector getCurrentVariableLowerBounds();
    DoubleVector getCurrentVariableUpperBounds();

  private:
};
