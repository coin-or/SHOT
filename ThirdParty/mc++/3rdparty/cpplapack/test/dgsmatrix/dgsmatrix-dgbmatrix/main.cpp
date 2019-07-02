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
  int M(5), N(5), CAP(4);
  int KL(1), KU(2);
  
  CPPL::dgsmatrix A(M,N,CAP);
  A.put(0,0, 1.);
  A.put(3,2, 2.);
  A.put(1,2, 3.);
  A.put(4,1, 4.);
  std::cout << "A =\n" << A << std::endl;
  
  CPPL::dgbmatrix B(M,N,KL,KU);
  for(int i=0; i<B.m; i++){
    for(int j=std::max(0l,i-B.kl); j<std::min(B.n,i+B.ku+1); j++){
      B(i,j) =double( rand() /(RAND_MAX/10) );
    }
  }
  std::cout << "B =\n" << B << std::endl;
  
  std::cout << "A+B =\n" << A+B << std::endl;
  std::cout << "A-B =\n" << A-B << std::endl;
  std::cout << "A*B =\n" << A*B << std::endl;
  
  return 0;
}

/*****************************************************************************/
