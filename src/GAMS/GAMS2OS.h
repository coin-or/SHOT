/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Shared.h"
#include "../SharedOS.h"
#include <cstdio>
#include <cstdlib>
#include <sys/stat.h> // for mkdir

#include "gmomcc.h"
#include "gevmcc.h"
#include "GamsNLinstr.h"

namespace SHOT
{

class GAMS2OS
{
  private:
    gmoHandle_t gmo;
    gevHandle_t gev;
    bool createdtmpdir;

    EnvironmentPtr env;

    OSnLNode *parseGamsInstructions(int codelen,      /**< length of GAMS instructions */
                                    int *opcodes,     /**< opcodes of GAMS instructions */
                                    int *fields,      /**< fields of GAMS instructions */
                                    int constantlen,  /**< length of GAMS constants pool */
                                    double *constants /**< GAMS constants pool */
    );

  public:
    GAMS2OS(EnvironmentPtr envPtr);
    ~GAMS2OS();
    void readGms(const std::string &filename);
    void readCntr(const std::string &filename);
    void writeResult(OSResult &osresult);
    void writeResult();
    void clear();
    void createOSObjects();

    OSInstance *osinstance;
};
} // namespace SHOT