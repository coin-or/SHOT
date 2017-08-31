#pragma once

#include "OSInstance.h"

typedef void* gmoHandle_t;
typedef void* gevHandle_t;

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
