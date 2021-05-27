/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "IModelingSystem.h"

#include "../Environment.h"
#include "../Enums.h"
#include "../Structs.h"

#include <memory>
#include <string>

class OSiLReader;
class OSInstance;
class OSnl2OS;
class OSnLNode;
class OSoLReader;

namespace SHOT
{

class NonlinearExpression;
using NonlinearExpressionPtr = std::shared_ptr<NonlinearExpression>;

enum class E_OSInputFileFormat
{
    OSiL,
    Ampl
};

class ModelingSystemOS : public IModelingSystem
{
public:
    ModelingSystemOS(EnvironmentPtr envPtr);
    ~ModelingSystemOS() override;

    // Adds modeling system specific settings
    static void augmentSettings(SettingsPtr settings);

    // Get specific settings from modeling system
    void updateSettings(SettingsPtr settings) override;

    // Create the optimization problem by filename in either OSiL or Ampl format
    E_ProblemCreationStatus createProblem(
        ProblemPtr& problem, const std::string& filename, const E_OSInputFileFormat& fileformat);

    // Create the optimization problem from an OSInstance
    E_ProblemCreationStatus createProblem(ProblemPtr& problem, std::shared_ptr<OSInstance> instance);

    // Move the solution and statistics from SHOT to the modeling system
    void finalizeSolution() override;

    std::shared_ptr<OSInstance> originalInstance;

private:
    OSInstance* readInstanceFromOSiL(const std::string& text);
    OSInstance* readInstanceFromOSiLFile(const std::string& filename);
    OSInstance* readInstanceFromAmplFile(const std::string& filename);

    bool copyVariables(OSInstance* source, ProblemPtr destination);
    bool copyObjectiveFunction(OSInstance* source, ProblemPtr destination);
    bool copyConstraints(OSInstance* source, ProblemPtr destination);
    bool copyLinearTerms(OSInstance* source, ProblemPtr destination);
    bool copyQuadraticTerms(OSInstance* source, ProblemPtr destination);
    bool copyNonlinearExpressions(OSInstance* source, ProblemPtr destination);
    NonlinearExpressionPtr convertOSNonlinearNode(OSnLNode* node, const ProblemPtr& destination);

    bool isObjectiveGenerallyNonlinear(OSInstance* instance);
    bool isObjectiveQuadratic(OSInstance* instance);

    std::vector<std::shared_ptr<OSiLReader>> osilReaders;
    std::shared_ptr<OSnl2OS> nl2os;

    // Determines whether all individual constraints are linear, quadratic or nonlinear
    std::vector<E_ConstraintClassification> getConstraintClassifications(OSInstance* instance);
};

using ModelingSystemOSPtr = std::shared_ptr<ModelingSystemOS>;

} // namespace SHOT