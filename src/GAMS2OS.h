#pragma once

#include "OSInstance.h"

#include "gmomcc.h"
#include "gevmcc.h"

class GAMS2OS
{
   private:
      gmoHandle_t gmo;
      gevHandle_t gev;

   public:
      GAMS2OS();
      void readGms(const std::string& filename);
      void createOSObjects();

      OSInstance* osinstance;
};
