/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include <cstdio>
#include <cstdlib>
#include <sys/stat.h> // for mkdir
#include "boost/filesystem.hpp"
#include "CoinHelperFunctions.hpp" // for CoinCopyOfArrayOrZero, maybe should eliminate this
#include "../ProcessInfo.h"

#include "OSInstance.h"
#include "OSResult.h"
#include "OSOption.h"
#include "OSoLReader.h"

#include "gmomcc.h"
#include "gevmcc.h"
#include "GamsNLinstr.h"

class ProcessInfo;

class GAMSOSInstance : public OSInstance
{
  public:
    gmoHandle_t gmo;

    GAMSOSInstance(gmoHandle_t gmo_) : gmo(gmo_), OSInstance()
    {
    }
};

class GAMS2OS
{
  private:
    gmoHandle_t gmo;
    gevHandle_t gev;
    bool createdtmpdir;

    OSnLNode *parseGamsInstructions(int codelen,      /**< length of GAMS instructions */
                                    int *opcodes,     /**< opcodes of GAMS instructions */
                                    int *fields,      /**< fields of GAMS instructions */
                                    int constantlen,  /**< length of GAMS constants pool */
                                    double *constants /**< GAMS constants pool */
    );

  public:
    GAMS2OS();
    ~GAMS2OS();
    void readGms(const std::string &filename);
    void readCntr(const std::string &filename);
    void writeResult(OSResult &osresult);
    void writeResult(ProcessInfo &info);
    void clear();
    void createOSObjects();

    OSInstance *osinstance;
};
