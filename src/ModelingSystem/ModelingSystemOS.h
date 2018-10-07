/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

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
#include "../SHOTSettings.h"

#include "../Model/ModelShared.h"
#include "../Model/Problem.h"

#include "OSInstance.h"
#include "OSiLWriter.h"
#include "OSiLReader.h"
#include "OSnl2OS.h"
#include "boost/filesystem.hpp"

#include "../ProcessInfo.h"

#include "IModelingSystem.h"

#include "vector"

namespace SHOT
{

typedef std::shared<OSInstance> OSInstancePtr;

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
    E_ProblemCreationStatus createProblem(ProblemPtr &problem, const std::string &filename, E_OSInputFileFormat fileformat);

    // Create the optimization problem from an OSInstance
    E_ProblemCreationStatus createProblem(ProblemPtr &problem, const OSInstancePtr &instance);

    // Move the solution and statistics from SHOT to the modeling system
    virtual void finalizeSolution();

  protected:
    virtual OSInstancePtr readInstanceFromOSiL(const std::string &text);
    virtual OSInstancePtr readInstanceFromOSiLFile(const std::string &filename);
    virtual OSInstancePtr readInstanceFromAmplFile(const std::string &filename);

    virtual bool copyVariables(OSInstancePtr source, ProblemPtr destination);

    virtual bool copyObjectiveFunction(OSInstancePtr source, ProblemPtr destination);

    virtual bool copyConstraints(OSInstancePtr source, ProblemPtr destination);

  private:
    std::vector<std::unique_ptr<OSiLReader>> osilReaders;
    std::unique_ptr<OSiLWriter> osilWriter;
    std::unique_ptr<OSnl2OS> nl2os;
};
} // namespace SHOT