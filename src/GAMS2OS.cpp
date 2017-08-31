#include "GAMS2OS.h"

#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>   // for mkdir

// set to 3 to see gams log
#define GAMSLOGOPTION 0

GAMS2OS::GAMS2OS()
: gmo(NULL), gev(NULL), osinstance(NULL)
{

}

void GAMS2OS::readGms(const std::string& filename)
{
   char gamscall[1024];
   char buffer[GMS_SSSIZE];
   int rc;
   FILE* convertdopt;

   assert(gmo == NULL);
   assert(gev == NULL);

   /* create temporary directory */
   mkdir("loadgms.tmp", S_IRWXU);

   /* create empty convertd options file */
   convertdopt = fopen("loadgms.tmp/convertd.opt", "w");
   if( convertdopt == NULL )
   {
      throw std::logic_error("Could not create convertd options file.");
   }
   fputs(" ", convertdopt);
   fclose(convertdopt);

   /* call GAMS with convertd solver to get compiled model instance in temporary directory */
   snprintf(gamscall, sizeof(gamscall), GAMSDIR "/gams %s SOLVER=CONVERTD SCRDIR=loadgms.tmp output=loadgms.tmp/listing optdir=loadgms.tmp optfile=1 pf4=0 solprint=0 limcol=0 limrow=0 pc=2 lo=%d", filename.c_str(), GAMSLOGOPTION);
   /* printf(gamscall); fflush(stdout); */
   rc = system(gamscall);
   if( rc != 0 )
   {
      snprintf(buffer, sizeof(buffer), "GAMS call returned with code %d", rc);
      throw std::logic_error(buffer);
   }

   /* initialize GMO and GEV libraries */
   if( !gmoCreateDD(&gmo, GAMSDIR, buffer, sizeof(buffer)) || !gevCreateDD(&gev, GAMSDIR, buffer, sizeof(buffer)) )
      throw std::logic_error(buffer);

   /* load control file */
   if( gevInitEnvironmentLegacy(gev, "loadgms.tmp/gamscntr.dat") )
   {
      gmoFree(&gmo);
      gevFree(&gev);
      throw std::logic_error("Could not load control file loadgms.tmp/gamscntr.dat.");
   }

   if( gmoRegisterEnvironment(gmo, gev, buffer) )
   {
      gmoFree(&gmo);
      gevFree(&gev);
      snprintf(buffer, sizeof(buffer), "Error registering GAMS Environment: %s", buffer);
      throw std::logic_error(buffer);
   }

   if( gmoLoadDataLegacy(gmo, buffer) )
   {
      gmoFree(&gmo);
      gevFree(&gev);
      throw std::logic_error("Could not load model data.");
   }


   //std::cout << "Number of variables: " << gmoN(gmo) << std::endl;
}

void GAMS2OS::createOSObjects()
{
   /* reformulate objective variable out of model, if possible */
   gmoObjStyleSet(gmo, gmoObjType_Fun);
   gmoObjReformSet(gmo, 1);
   gmoIndexBaseSet(gmo, 0);

   throw std::logic_error("createOSObjects not implemented yet");
}
