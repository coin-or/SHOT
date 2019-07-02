/*****************************************************************************/
/*                                 noname                                    */
/*****************************************************************************/

//=============================================================================
#include <ctime>
#include "cpplapack.h"

//=============================================================================
/*! main */
int main(int argc, char** argv)
{
  srand(time(NULL));
  int N(5);
  
  CPPL::dssmatrix A(N);
  A.put(0,0, 1.);
  A.put(3,2, 2.);
  A.put(1,2, 3.);
  A.put(4,1, 4.);
  std::cout << "A =\n" << A << std::endl;
  for(std::vector<CPPL::dcomponent>::const_iterator it=A.data.begin(); it!=A.data.end(); it++){
    std::cout << "A(" << it->i << "," << it->j << ") =" << it->v << std::endl;
  }
  
  CPPL::dcovector x(N);
  for(int i=0; i<x.l; i++){
    x(i) =double( rand() /(RAND_MAX/10) );
  }
  std::cout << "x =\n" << x << std::endl;
  
  std::cout << "A*x =\n" << A*x << std::endl;
  
  CPPL::dsymatrix B(A.to_dsymatrix());
  std::cout << "B =\n" << B << std::endl;
  std::cout << "B*x =\n" << B*x << std::endl;
  
  return 0;
}

/*****************************************************************************/
