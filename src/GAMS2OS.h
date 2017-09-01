#pragma once

#include "OSInstance.h"

#include "gmomcc.h"
#include "gevmcc.h"
#include "GamsNLinstr.h"

class GAMSOSInstance : public OSInstance
{
public:
    gmoHandle_t gmo;

    GAMSOSInstance(gmoHandle_t gmo_)
    : gmo(gmo_),
	  OSInstance()
    { }

};

class GAMS2OS
{
   private:
      gmoHandle_t gmo;
      gevHandle_t gev;

      OSnLNode* parseGamsInstructions(
         int                codelen,            /**< length of GAMS instructions */
         int*               opcodes,            /**< opcodes of GAMS instructions */
         int*               fields,             /**< fields of GAMS instructions */
         int                constantlen,        /**< length of GAMS constants pool */
         double*            constants           /**< GAMS constants pool */
		 );

   public:
      GAMS2OS();
      void readGms(const std::string& filename);
      void createOSObjects();

      OSInstance* osinstance;
};
