/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Environment.h"
#include "../Enums.h"
#include "../Structs.h"

namespace SHOT
{

enum class E_ProblemCreationStatus
{
    NormalCompletion,
    FileDoesNotExist,
    ErrorInFile,
    CapabilityProblem,
    ErrorInVariables,
    ErrorInConstraints,
    ErrorInObjective,
    Error
};

class IModelingSystem
{
public:
    IModelingSystem(EnvironmentPtr envPtr) : env(envPtr) {};
    virtual ~IModelingSystem() = default;

    // Adds modeling system specific settings
    static void augmentSettings([[maybe_unused]] SettingsPtr settings) {};

    // Get specific settings from modeling system
    virtual void updateSettings(SettingsPtr settings) = 0;

    // Move the solution and statistics from SHOT to the modeling system
    virtual void finalizeSolution() = 0;

protected:
    EnvironmentPtr env;
};
} // namespace SHOT