/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "IModelingSystem.h"

namespace SHOT
{

//typedef std::shared_ptr<OSInstance> OSInstance *;

enum class E_OSInputFileFormat
{
    OSiL,
    Ampl
};

class ModelingSystemOS : public IModelingSystem
{
  public:
    ModelingSystemOS(EnvironmentPtr envPtr);
    virtual ~ModelingSystemOS();

    // Adds modeling system specific settings
    virtual void augmentSettings(SettingsPtr settings);

    // Get specific settings from modeling system
    virtual void updateSettings(SettingsPtr settings);

    // Create the optimization problem by filename in either OSiL or Ampl format
    E_ProblemCreationStatus createProblem(ProblemPtr &problem, const std::string &filename, const E_OSInputFileFormat &fileformat);

    // Create the optimization problem from an OSInstance
    E_ProblemCreationStatus createProblem(ProblemPtr &problem, OSInstance *instance);

    // Move the solution and statistics from SHOT to the modeling system
    virtual void finalizeSolution();

    OSInstance *originalInstance;

  private:
    OSInstance *readInstanceFromOSiL(const std::string &text);
    OSInstance *readInstanceFromOSiLFile(const std::string &filename);
    OSInstance *readInstanceFromAmplFile(const std::string &filename);

    bool copyVariables(OSInstance *source, ProblemPtr destination);
    bool copyObjectiveFunction(OSInstance *source, ProblemPtr destination);
    bool copyConstraints(OSInstance *source, ProblemPtr destination);
    bool copyLinearTerms(OSInstance *source, ProblemPtr destination);
    bool copyQuadraticTerms(OSInstance *source, ProblemPtr destination);
    bool copyNonlinearExpressions(OSInstance *source, ProblemPtr destination);
    NonlinearExpressionPtr convertOSNonlinearNode(OSnLNode *node, const ProblemPtr &destination);

    bool isObjectiveGenerallyNonlinear(OSInstance *instance);
    bool isObjectiveQuadratic(OSInstance *instance);

    std::vector<std::shared_ptr<OSiLReader>> osilReaders;
    std::shared_ptr<OSiLWriter> osilWriter;
    std::shared_ptr<OSnl2OS> nl2os;

    //bool areAllConstraintsLinear(OSInstance* instance);
    //bool areAllConstraintsQuadratic(OSInstance* instance);
    //bool areAllVariablesReal(OSInstance* instance);

    //bool isConstraintNonlinear(OSInstance* instance);
    //bool isConstraintQuadratic(OSInstance* instance);

    // Determines whether all individual constraints are linear, quadratic or nonlinear
    std::vector<E_ConstraintClassification> getConstraintClassifications(OSInstance *instance);
};

typedef std::shared_ptr<ModelingSystemOS> ModelingSystemOSPtr;

} // namespace SHOT