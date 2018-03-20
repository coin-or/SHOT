/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "Enums.h"
#include "SHOTSettings.h"
#include "SolutionStrategy/ISolutionStrategy.h"
#include "SolutionStrategy/SolutionStrategySingleTree.h"
#include "SolutionStrategy/SolutionStrategyMultiTree.h"
#include "SolutionStrategy/SolutionStrategyMIQCQP.h"
#include "SolutionStrategy/SolutionStrategyNLP.h"
#include "ProcessInfo.h"
#include "boost/filesystem.hpp"
#include "TaskHandler.h"
#include "OSnl2OS.h"

#ifdef HAS_GAMS
#include "GAMS/GAMS2OS.h"
#endif

class SHOTSolver
{
  private:
    unique_ptr<ISolutionStrategy> solutionStrategy;

#ifdef HAS_GAMS
    unique_ptr<GAMS2OS> gms2os;
#endif

    void initializeSettings();
    void verifySettings();

    void initializeDebugMode();

    bool isProblemInitialized;

    //OSiLReader *osilReader = NULL;

  public:
    SHOTSolver();
    ~SHOTSolver();

    bool setOptions(std::string fileName);
    bool setOptions(OSOption *osOptions);

    bool setProblem(std::string fileName);
    bool setProblem(OSInstance *osInstance);

    bool solveProblem();

    std::string getOSoL();
    std::string getGAMSOptFile();

    std::string getOSrL();
    std::string getTraceResult();
};
