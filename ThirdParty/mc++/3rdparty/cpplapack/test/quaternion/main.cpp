#include "cpplapack.h"

//=============================================================================
/*! main */
int main(int argc, char** argv)
{
  CPPL::dquater q( vt2q(CPPL::dcovec3(1,1,1),M_PI/3.) );
  std::cout << t(q) << std::endl;
  std::cout << t(q*inv(q)) << std::endl;
  
  std::cout << q2m(q) << std::endl;
  
  //// dcovec3 ////
  std::cout << t(rotate(CPPL::dcovec3(1,0,0),q)) << std::flush;
  std::cout << t(rotate(CPPL::dcovec3(0,1,0),q)) << std::flush;
  std::cout << t(rotate(CPPL::dcovec3(0,0,1),q)) << std::flush;
  
  //// dgemat3 ////
  CPPL::dgemat3 gm;
  gm.identity();
  std::cout << rotate(gm,q) << std::endl;
  
  //// dsymat3 ////
  CPPL::dsymat3 sm;
  sm.identity();
  std::cout << rotate(sm,q) << std::endl;
  
  return 0;
}
