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

#include <string>

namespace SHOT
{

class ModelingSystemAMPL : public IModelingSystem
{
public:
    ModelingSystemAMPL(EnvironmentPtr envPtr);
    ~ModelingSystemAMPL() override;

    // Adds modeling system specific settings
    static void augmentSettings(SettingsPtr settings);

    // Get specific settings from modeling system
    void updateSettings(SettingsPtr settings) override;

    // Create the optimization problem by filename in AMPL format
    E_ProblemCreationStatus createProblem(ProblemPtr& problem, const std::string& filename);

    // Move the solution and statistics from SHOT to the modeling system
    void finalizeSolution() override;

protected:
private:
};
} // namespace SHOT