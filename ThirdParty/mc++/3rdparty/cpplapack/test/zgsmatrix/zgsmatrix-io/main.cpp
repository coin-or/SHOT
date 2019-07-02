/*****************************************************************************/
/*                                 noname                                    */
/*****************************************************************************/

//====================================================================[include]
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "cpplapack.h"
using namespace std;

//=======================================================================[main]
/*! main */
int main(int argc, char** argv)
{
  srand(time(NULL));
  int M(5), N(3), CAP(4);
  
  CPPL::zgsmatrix A(M,N,CAP);
  A.put(0,0, complex<double>(1.,2.) );
  A.put(3,2, complex<double>(3.,4.) );
  A.put(1,2, complex<double>(5.,6.) );
  A.put(4,1, complex<double>(7.,8.) );
  
  cout << "A =\n" << A << endl;
  
  //A.put(1,2, 4.5);
  //A.add(1,2, 0.1);
  //A.sub(1,2, 0.1);
  //A.mult(1,2, 10.);
  //A.div(1,2, 10.);
  //A.del(1,2);
  A.del(2);
  cout << "A =\n" << A << endl;
  
  const CPPL::zgsmatrix B(A);
  //// write/read ////
  B.write( "tmp.txt" );
  
  CPPL::zgsmatrix C;
  C.read( "tmp.txt" );
  cout << "C-B =\n" << C-B << "<-Should be zero." << endl;
  
  return 0;
}

/*****************************************************************************/
