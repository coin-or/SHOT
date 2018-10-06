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
  int M(5), N(3), CAP(4);
  
  CPPL::dgsmatrix A(M,N,CAP);
  A.put(0,0, 1.);
  A.put(3,2, 2.);
  A.put(1,2, 3.);
  A.put(4,1, 4.);
  
  std::cout << "A =\n" << A << std::endl;
  for(std::vector<CPPL::dcomponent>::const_iterator it=A.data.begin(); it!=A.data.end(); it++){
    std::cout << "A(" << it->i << "," << it->j << ") = " << it->v << std::endl;
  }

  
  //A.put(1,2, 4.5);
  //A.add(1,2, 0.1);
  //A.sub(1,2, 0.1);
  //A.mult(1,2, 10.);
  //A.div(1,2, 10.);
  //A.del(1,2);
  A.del(2);
  std::cout << "A =\n" << A << std::endl;
  
  const CPPL::dgsmatrix B(A);
  //// write/read ////
  B.write( "tmp.txt" );
  
  CPPL::dgsmatrix C;
  C.read( "tmp.txt" );
  std::cout << "C-B =\n" << C-B << "<-Should be zero." << std::endl;
  
  return 0;
}

/*****************************************************************************/
