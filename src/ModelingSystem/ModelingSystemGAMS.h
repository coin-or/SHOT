/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.
   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Enums.h"
#include "../Structs.h"
#include "../Environment.h"
#include "../Output.h"
#include "../SHOTSettings/SHOTSettings.h"

#include "../Model/Problem.h"

#include "boost/filesystem.hpp"

#include "../ProcessInfo.h"

#include "IModelingSystem.h"

#include <cstdio>
#include <cstdlib>
#include <sys/stat.h> // for mkdir

#include "gmomcc.h"
#include "gevmcc.h"
#include "GamsNLinstr.h"

#include <vector>
#include <memory>

namespace SHOT
{

enum class E_GAMSInputSource
{
    ProblemFile,
    GAMSModel
};

class ModelingSystemGAMS : public IModelingSystem
{
  public:
    ModelingSystemGAMS(EnvironmentPtr envPtr);
    virtual ~ModelingSystemGAMS();

    // Adds modeling system specific settings
    virtual void augmentSettings(SettingsPtr settings);

    // Get specific settings from modeling system
    virtual void updateSettings(SettingsPtr settings);

    // Create the optimization problem by filename either directly from a gms-file or from a compiled GAMS model
    E_ProblemCreationStatus createProblem(ProblemPtr &problem, const std::string &filename, const E_GAMSInputSource &inputSource);

    // Move the solution and statistics from SHOT to the modeling system
    virtual void finalizeSolution();

  private:
    gmoHandle_t gmo;
    gevHandle_t gev;
    bool createdtmpdir;
    char buffer[GMS_SSSIZE];

    void createModelFromProblemFile(const std::string &filename);
    void createModelFromGAMSModel(const std::string &filename);

    void clearGAMSObjects();

    /*OSInstance *readInstanceFromOSiL(const std::string &text);
    OSInstance *readInstanceFromOSiLFile(const std::string &filename);
    OSInstance *readInstanceFromAmplFile(const std::string &filename);*/

    bool copyVariables(ProblemPtr destination);
    bool copyObjectiveFunction(ProblemPtr destination);
    bool copyConstraints(ProblemPtr destination);
    bool copyLinearTerms(ProblemPtr destination);
    bool copyQuadraticTerms(ProblemPtr destination);
    bool copyNonlinearExpressions(ProblemPtr destination);
    //NonlinearExpressionPtr convertOSNonlinearNode(const ProblemPtr &destination);

    static void applyOperation(std::vector<NonlinearExpressionPtr> &stack, NonlinearExpressionPtr op, int nargs);

    NonlinearExpressionPtr parseGamsInstructions(int codelen,       /**< length of GAMS instructions */
                                                 int *opcodes,      /**< opcodes of GAMS instructions */
                                                 int *fields,       /**< fields of GAMS instructions */
                                                 int constantlen,   /**< length of GAMS constants pool */
                                                 double *constants, /**< GAMS constants pool */
                                                 const ProblemPtr &destination);

    /*std::vector<std::shared_ptr<OSiLReader>> osilReaders;
    std::shared_ptr<OSiLWriter> osilWriter;
    std::shared_ptr<OSnl2OS> nl2os;
    */

    //bool areAllConstraintsLinear(OSInstance* instance);
    //bool areAllConstraintsQuadratic(OSInstance* instance);
    //bool areAllVariablesReal(OSInstance* instance);

    //bool isConstraintNonlinear(OSInstance* instance);
    //bool isConstraintQuadratic(OSInstance* instance);

    // Determines whether all individual constraints are linear, quadratic or nonlinear
    //std::vector<E_ConstraintClassification> getConstraintClassifications(OSInstance *instance);
};

typedef std::shared_ptr<ModelingSystemGAMS> ModelingSystemGAMSPtr;

} // namespace SHOT