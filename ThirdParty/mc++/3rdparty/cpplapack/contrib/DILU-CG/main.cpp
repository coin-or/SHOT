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
CPPL::dcovector solve
(
 const CPPL::dsymatrix& A,
 const CPPL::dcovector& inv_D,
 const CPPL::dcovector& y
 )
{
  CPPL::dcovector z(y.l);
  for(long i=0; i<y.l; i++){
    double sum_lz(0.0);
    for(long j=0; j<i; j++){ sum_lz +=A(i,j)*z(j); }
    z(i) =inv_D(i)*(y(i)-sum_lz);
  }
  
  CPPL::dcovector x(y.l);
  for(long i=y.l-1; i>=0; i--){
    double sum_ux(0.0);
    for(long j=y.l-1; j>i; j--){ sum_ux +=A(i,j)*x(j); }
    x(i) =z(i) -inv_D(i)*sum_ux;
  }
  
  return x;
}

//=============================================================================
bool dilucg
(
 const CPPL::dsymatrix& A, 
 CPPL::dcovector& x, 
 const double& eps
)
{
  CPPL::dcovector inv_D(x.l);
  for(long i=0; i<x.l; i++){ inv_D(i)=A(i,i); }
  for(long i=0; i<x.l; i++){
    inv_D(i) =1./inv_D(i);
    for(long j=i+1; j<x.l; j++){
      inv_D(j) -=pow(A(i,j),2)*inv_D(i);
    }
  }
  //inv_D.write("inv_D");
  
  double alpha, beta, rho(1.0), rho2;
  const CPPL::dcovector y(x);
  CPPL::dcovector z, r(x), p(x.l), q;
  x.zero(); p.zero();
  
  int itc(0);
  const int itmax(2*x.l);
  while(fabs(damax(r))>eps && ++itc<itmax){
    std::cout << itc << " "  << fabs(damax(r)) << " " << fabs(damax(y-A*x)) << " " << rho << std::endl;
    z =solve(A,inv_D,r);
    
    rho2=r%z;
    beta=rho2/rho;
    rho=rho2;
    
    p =z +beta*p;
    q =A*p;
    alpha=rho/(p%q);
    
    x+=alpha*p;
    r-=alpha*q;
  }
  std::cerr << "itc=" << itc << "  fabs(damax(r))=" << fabs(damax(r)) << std::endl;
  
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
  if( dilucg(A, y, eps) ){
    std::cerr << "failed." << std::endl;
    exit(1);
  }
  y.write("solution.dcovector");
  std::cout << "solution=\n" << t(y) << std::endl;

  return 0;
}
