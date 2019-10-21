/*****************************************************************************/
/*                                main.cpp                                   */
/*****************************************************************************/
//=============================================================================
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include "cpplapack.h"

//=============================================================================
bool minres
(
 const CPPL::dsymatrix& A,
 CPPL::dcovector& x,
 const double& eps
)
{
  const CPPL::dcovector y(x);
  CPPL::dcovector r(y);

  double beta2(nrm2(r)), beta3;
  double rho0(1.0), rho1(beta2), rho2;
  double rhop(0.0);
  double c0(0.0), c1(-1.0), c2;
  double s0(0.0), s1(0.0), s2;
  double f(beta2);

  CPPL::dcovector p1(x.l), p2(r/beta2), p3;
  CPPL::dcovector q0(x.l), q1(x.l), q2;
  x.zero();
  p1.zero();
  q0.zero();
  q1.zero();

  int itc(0);
  const int itmax(2*x.l);
  while( (fabs(f)>eps || fabs(damax(y-A*x))>eps) && itc<itmax){
    std::cout << itc << " " << fabs(damax(y-A*x)) << std::endl;
    //std::cerr << "itc=" << itc << ", fabs(f)=" << fabs(f) << std::endl;
    CPPL::dcovector Ap2(A*p2), z;
    z =Ap2-beta2*p1;
    double alpha;
    alpha =Ap2%p2;
    p3 =z-alpha*p2;
    beta3 =nrm2(p3);
    p3 /=beta3;

    double d, h;
    d =(alpha-rhop*c0)*s1;
    h =beta2*s0;

    rhop =-beta2*c0*s1 -alpha*c1;
    rho2 =sqrt(pow(rhop,2)+pow(beta3,2));
    c2 =rhop/rho2;
    s2 =beta3/rho2;

    CPPL::dcovector zp;
    zp =p2 -(h/rho0)*q0;
    q2 =zp -(d/rho1)*q1;
    double t;
    t =f*c2;
    f *=s2;

    x +=(t/rho2)*q2;
    beta2=beta3;
    rho0=rho1; rho1=rho2;
    c0=c1; c1=c2;
    s0=s1; s1=s2;
    swap(p1,p2); swap(p2,p3);
    swap(q0,q1); swap(q1,q2);

    itc++;
  }
  std::cerr << "itc=" << itc << "  fabs(damax(y-A*x))=" << fabs(damax(y-A*x)) << std::endl;
  //std::cerr << "itc=" << itc << "  fabs(f)=" << fabs(f) << std::endl;

  if(itc<itmax){ return 0; }
  else{ return 1; }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//=============================================================================
int main(int argc, char** argv)
{
  srand(time(NULL));
  
  const int size(100);
  CPPL::dsymatrix A(size);
  
  for(int i=0; i<size; i++){
    for(int j=0; j<=i; j++){
      A(i,j) =(double(rand())/double(RAND_MAX))*2.0 -1.0;
    }
    A(i,i)+=10.;
  }
  A.write("A.dsymatrix");
  
  CPPL::dcovector x(size);
  for(int i=0; i<size; i++){
    x(i) =(double(rand())/double(RAND_MAX))*1. -0.5;
  }
  x.write("answer.dcovector");//solution
  std::cerr << "answer=\n" << t(x) << std::endl;
  
  CPPL::dcovector y(size);
  y=A*x;
  y.write("y.dcovector");
  
  double eps(fabs(damax(y))*1e-6);
  //std::cerr << "eps=" << eps << std::endl;
  if( minres(A, y, eps) ){
    std::cerr << "failed." << std::endl;
    exit(1);
  }
  y.write("solution.dcovector");
  std::cout << "solution=\n" << t(y) << std::endl;
  
  return 0;
}
