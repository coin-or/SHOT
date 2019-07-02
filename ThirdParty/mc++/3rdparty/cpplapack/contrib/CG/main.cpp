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
bool cg
(
 //const CPPL::dsymatrix& A, 
 const CPPL::dssmatrix& A, 
 CPPL::dcovector& x, 
 const double& eps
)
{
  double alpha, beta, rho(1.0), rho2;
  const CPPL::dcovector y(x);
  CPPL::dcovector r(x), p(x.l), q;
  x.zero(); p.zero();

  int itc(0);
  const int itmax(2*x.l);
  while(fabs(damax(r))>eps && itc<itmax){
    std::cout << itc << " " << fabs(damax(y-A*x)) << std::endl;
    //std::cerr << "itc=" << itc << ", fabs(damax(r))=" << fabs(damax(r)) << std::endl;
    rho2=r%r;
    beta=rho2/rho;
    rho=rho2;
    
    p =r +beta*p;
    q=A*p;
    alpha=rho/(p%q);
    
    x+=alpha*p;
    r-=alpha*q;
    
    itc++;
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
  //CPPL::dsymatrix A(size);
  CPPL::dssmatrix A(size);
  
  for(int i=0; i<size; i++){
    for(int j=0; j<=i; j++){
      if(rand()%2){ A(i,j) =(double(rand())/double(RAND_MAX))*2.0 -1.0; }
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
  if( cg(A, y, eps) ){
    std::cerr << "failed." << std::endl;
    exit(1);
  }
  y.write("solution.dcovector");
  std::cout << "solution=\n" << t(y) << std::endl;
  
  return 0;
}
