/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Enums.h"
#include "Structs.h"
#include "Environment.h"

#include "OSiLWriter.h"
#include "OSiLReader.h"

namespace SHOT
{
class Model
{
  public:
    Model(EnvironmentPtr envPtr);
    ~Model();

    OriginalProblemPtr originalProblem;

    ModelStatistics statistics;

    PairDouble currentObjectiveBounds;

    void setStatistics();

    PairDouble getCorrectedObjectiveBounds();

    OSInstance *getProblemInstanceFromOSiL(std::string osil);
    std::string getOSiLFromProblemInstance(OSInstance *instance);

  private:
    EnvironmentPtr env;

    std::vector<OSiLReader *> osilReaders;
    OSiLWriter *osilWriter;
};
} // namespace SHOT