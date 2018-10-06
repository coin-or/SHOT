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
bool bicgstab
(
 const CPPL::dgematrix& A,
 CPPL::dcovector& x,
 const double& eps
)
{
  double alpha(0.0), zeta(1.0);
  CPPL::dcovector t, R(x), r(R), p(x.l), Ap(x.l), At;
  x.zero(); p.zero(); Ap.zero();

  int itc(0);
  const int itmax(2*x.l);
  while(fabs(damax(r))>eps && itc<itmax){
    std::cerr << "itc=" << itc << ", fabs(damax(r))=" << fabs(damax(r)) << std::endl;

    p =r +(alpha/zeta)*p -alpha*Ap;

    Ap =A*p;
    alpha =(R%r)/(R%Ap);
    t =r -alpha*Ap;
    At =A*t;
    zeta =(At%t)/(At%At);

    x +=alpha*p +zeta*t;
    r =t -zeta*At;
    alpha =(R%r)/(R%Ap);

    itc++;
  }
  std::cerr << "fabs(damax(r))=" << fabs(damax(r)) << std::endl;
  
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
  CPPL::dgematrix A(size,size);
  
  for(int i=0; i<size; i++){
    for(int j=0; j<size; j++){
      A(i,j) =(double(rand())/double(RAND_MAX))*2.0 -1.0;
    }
    A(i,i)+=10.;
  }
  A.write("A.dgematrix");
  
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
  if( bicgstab(A, y, eps) ){
    std::cerr << "failed." << std::endl;
    exit(1);
  }
  y.write("solution.dcovector");
  std::cout << "solution=\n" << t(y) << std::endl;
  
  return 0;
}
