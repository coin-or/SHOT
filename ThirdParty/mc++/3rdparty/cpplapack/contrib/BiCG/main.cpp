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
bool bicg
(
 const CPPL::dgematrix& A,
 //const CPPL::dgsmatrix& A,
 CPPL::dcovector& x,
 const double& eps
)
{
  double alpha, beta(0.0);
  CPPL::dcovector p_0(x.l), p_1, P_0(x.l), P_1, q, Q;
  CPPL::dcovector r_1(x), r_2, R_1(r_1), R_2;
  double rho_0(r_1%R_1), rho_1;
  x.zero();
  p_0.zero();
  P_0.zero();
  
  int itc(0);
  const int itmax(2*x.l);
  while(fabs(damax(r_1))>eps && ++itc<itmax){
    std::cout << itc << " " << fabs(damax(r_1)) << std::endl;
    
    rho_1 =r_1%R_1;
    beta =rho_1/rho_0;
    p_1 =r_1 +beta*p_0;
    P_1 =R_1 +beta*P_0;
    q =A*p_1;
    Q =t(t(P_1)*A);
    alpha =rho_1/(P_1%q);
    x +=alpha*p_1;
    r_2 =r_1 -alpha*q;
    R_2 =R_1 -alpha*Q;

    rho_0 =rho_1;
    swap(p_0, p_1);
    swap(P_0, P_1);
    swap(r_1, r_2);
    swap(R_1, R_2);
  }
  std::cerr << "itc=" << itc << "  fabs(damax(r_1))=" << fabs(damax(r_1)) << std::endl;
  
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
  //CPPL::dgsmatrix A(size,size);
  
  for(int i=0; i<size; i++){
    for(int j=0; j<size; j++){
      if(rand()%2){ A(i,j) =(double(rand())/double(RAND_MAX))*2.0 -1.0; }
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
  
  CPPL::dcovector y(A*x);
  y.write("y.dcovector");
  //std::cerr << "y=\n" << t(y) << std::endl;
  
  double eps(fabs(damax(y))*1e-6);
  //std::cerr << "eps=" << eps << std::endl;
  if( bicg(A, y, eps) ){
    std::cerr << "failed." << std::endl;
    exit(1);
  }
  y.write("solution.dcovector");
  std::cout << "solution=\n" << t(y) << std::endl;
  //std::cerr << "A*x=\n" << t(A*y) << std::endl;
  
  return 0;
}
