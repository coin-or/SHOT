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
  A(0,0)=1;
  A(2,3)=2;
  A(1,2)=3;
  A(4,1)=4;
  //A(3,2)+=0.1;
  
  std::cout << "A =\n" << A << std::endl;
  std::cout << "A(1,1)=" << A(1,1) << std::endl;
  for(std::vector<CPPL::dcomponent>::const_iterator it=A.data.begin(); it!=A.data.end(); it++){
    std::cout << "A(" << it->i << "," << it->j << ") =" << it->v << std::endl;
  }
  A.checkup();
  
  A.put(3,4, 3.4);
  A.del(1,4);
  A.del(0);
  std::cout << "A =\n" << A << std::endl;
  
  const CPPL::dssmatrix B(A);
  //// write/read ////
  B.write( "tmp.txt" );
  
  CPPL::dssmatrix C;
  C.read( "tmp.txt" );
  std::cout << "C-B =\n" << C-B << "<-Should be zero." << std::endl;
  
  return 0;
}

/*****************************************************************************/
