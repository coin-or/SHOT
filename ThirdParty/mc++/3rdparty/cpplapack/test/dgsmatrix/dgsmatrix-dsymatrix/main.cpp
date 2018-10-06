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
  
  CPPL::dgsmatrix A(M,N,CAP);
  A.put(0,0, 1.);
  A.put(3,2, 2.);
  A.put(1,2, 3.);
  A.put(4,1, 4.);
  std::cout << "A =\n" << A << std::endl;
  
  CPPL::dsymatrix B(N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<=i; j++){
    B(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  std::cout << "B =\n" << B << std::endl;
  
  //std::cout << "A+B =\n" << A+B << std::endl;
  std::cout << "A-B =\n" << A-B << std::endl;
  std::cout << "A*B =\n" << A*B << std::endl;
  
  return 0;
}

/*****************************************************************************/
