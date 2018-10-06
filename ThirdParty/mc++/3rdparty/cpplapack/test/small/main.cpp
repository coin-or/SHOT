#include "cpplapack.h"

//=============================================================================
/*! main */
int main(int argc, char** argv)
{
  srand(time(NULL));
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  CPPL::dcovec2 cv2;
  CPPL::drovec2 rv2;
  CPPL::dgemat2 gm2;
  CPPL::dsymat2 sm2;
  
  gm2.identity();
  std::cout << "gm2=\n" << gm2 << std::endl;
  std::cout << "t2m(0.1)*gm2=\n" << CPPL::t2m(0.1)*gm2 << std::endl;

  gm2(0,0) =1.; gm2(0,1) =2.;
  gm2(1,0) =3.; gm2(1,1) =4.;
  std::cout << "gm2=\n" << gm2 << std::endl;
  std::cout << "inv(gm2)=\n" << inv(gm2) << std::endl;
  std::cout << "gm2*inv(gm2)=\n" << gm2*inv(gm2) << std::endl;
  std::cout << "rotate(gm2,0.1)=\n" << rotate(gm2,0.1) << std::endl;
  
  sm2(0,0) =1.;
  sm2(1,0) =2.; sm2(1,1) =3.;
  std::cout << "sm2=\n" << sm2 << std::endl;
  std::cout << "inv(sm2)=\n" << inv(sm2) << std::endl;
  std::cout << "sm2*inv(sm2)=\n" << sm2*inv(sm2) << std::endl;
  std::cout << "rotate(sm2,0.1)=\n" << rotate(sm2,0.1) << std::endl;
  
  
  cv2 += t(rv2);
  
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  CPPL::dcovec3 cv3;
  CPPL::drovec3 rv3;
  CPPL::dgemat3 gm3;
  CPPL::dsymat3 sm3;
  CPPL::dquater q;
  
  gm3.identity();
  //std::cout << "gm3=\n" << gm3 << std::endl;
  //std::cout << "vt2q(0.1)*gm2=\n" << CPPL::t2m(0.1)*gm2 << std::endl;
  
  sm3(0,0)=1.;
  sm3(1,0)=2.; sm3(1,1)=3.;
  sm3(2,0)=4.; sm3(2,1)=5.; sm3(2,2)=6.;
  std::cout << "sm3=\n" << sm3 << std::endl;
  std::cout << "inv(sm3)=\n" << inv(sm3) << std::endl;
  std::cout << "sm3*inv(sm3)=\n" << sm3*inv(sm3) << std::endl;
  
  return 0;
}
