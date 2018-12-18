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

namespace SHOT
{
class NLPSolverGAMS : public NLPSolverBase
{
  private:
    gmoHandle_t modelingObject;
    gevHandle_t modelingEnvironment;

    char nlpsolver[GMS_SSSIZE];
    char nlpsolveropt[GMS_SSSIZE];
    double timelimit;
    int iterlimit;
    bool showlog;

  public:
    NLPSolverGAMS(EnvironmentPtr envPtr, gmoHandle_t modelingObject);

    virtual ~NLPSolverGAMS();

    void setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues);
    void clearStartingPoint();

    void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues);

    void unfixVariables();

    void saveOptionsToFile(std::string fileName);

    void saveProblemToFile(std::string fileName);

    VectorDouble getSolution();
    double getSolution(int i);

    virtual double getObjectiveValue();

    virtual void updateVariableLowerBound(int variableIndex, double bound);
    virtual void updateVariableUpperBound(int variableIndex, double bound);

  protected:
    E_NLPSolutionStatus solveProblemInstance();

    VectorDouble getVariableLowerBounds();
    VectorDouble getVariableUpperBounds();

  private:
};
} // namespace SHOT