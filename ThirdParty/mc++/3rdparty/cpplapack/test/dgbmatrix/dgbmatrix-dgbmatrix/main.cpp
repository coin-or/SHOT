/*****************************************************************************/
/*                                 noname                                    */
/*****************************************************************************/

//=============================================================================
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "cpplapack.h"

//=============================================================================
/*! main */
int main(int argc, char** argv)
{
  srand(time(NULL));
  long M(4), N(3), KL(2), KU(1);
  //long M(10), N(4), KL(4), KU(0);
  //long M(2), N(2), KL(0), KU(0);
  
  CPPL::dgbmatrix A(M,N,KL,KU);
  for(long i=0; i<A.m; i++){
    for(long j=std::max(long(0),i-A.kl); j<std::min(A.n,i+A.ku+1); j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
    }
  }
  std::cout << "A =\n" << A << std::endl;
  
  CPPL::dgbmatrix B;
  std::cout << "#### B=A; ####" << std::endl;
  B=A;
  std::cout << "B =\n" << B << std::endl;
  
  std::cout << "#### B+=A; ####" << std::endl;
  B+=A;
  std::cout << "B =\n" << B << std::endl;

  std::cout << "#### B-=A; ####" << std::endl;
  B-=A;
  std::cout << "B =\n" << B << std::endl;
  
  std::cout << "A+B =\n" << A+B << std::endl;
  std::cout << "A-B =\n" << A-B << std::endl;
  
  CPPL::dgematrix A2( A.to_dgematrix() ), B2( B.to_dgematrix() );
  
  std::cout << "A*t(B) =\n" << A*t(B) << std::endl;
  std::cout << "A2*t(B2) =\n" << A2*t(B2) << std::endl;
  
  CPPL::dgbmatrix P(8,10,2,3), Q(10,9,1,3), R;
  for(long i=0; i<P.m; i++){
    for(long j=std::max(long(0),i-P.kl); j<std::min(P.n,i+P.ku+1); j++){
      P(i,j) =double( rand() /(RAND_MAX/9) +1 );
    }
  }
  for(long i=0; i<Q.m; i++){
    for(long j=std::max(long(0),i-Q.kl); j<std::min(Q.n,i+Q.ku+1); j++){
      Q(i,j) =double( rand() /(RAND_MAX/9) +1 );
    }
  }
  CPPL::dgematrix P2( P.to_dgematrix() ), Q2( Q.to_dgematrix() );
  std::cout << "P =\n" << P << std::endl;
  std::cout << "P2 =\n" << P2 << std::endl;
  std::cout << "Q =\n" << Q << std::endl;
  std::cout << "Q2 =\n" << Q2 << std::endl;
  std::cout << "P*Q =\n" << P*Q << std::endl;
  std::cout << "P2*Q2 =\n" << P2*Q2 << std::endl;
  
  std::cout << "#### P*=Q; ####" << std::endl;
  P*=Q;
  std::cout << "P =\n" << P << std::endl;
  
  return 0;
}

/*****************************************************************************/
