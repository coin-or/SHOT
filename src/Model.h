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

class Model
{
  public:
    Model(EnvironmentPtr envPtr);
    ~Model();

    OriginalProblemPtr originalProblem;

    ModelStatistics statistics;

    DoublePair currentObjectiveBounds;

    void setOriginalProblem(OriginalProblemPtr problem);
    void setStatistics();

    DoublePair getCorrectedObjectiveBounds();

    OSInstance *getProblemInstanceFromOSiL(std::string osil);
    std::string getOSiLFromProblemInstance(OSInstance *instance);

  private:
    EnvironmentPtr env;

    std::vector<OSiLReader *> osilReaders;
    OSiLWriter *osilWriter;
};
