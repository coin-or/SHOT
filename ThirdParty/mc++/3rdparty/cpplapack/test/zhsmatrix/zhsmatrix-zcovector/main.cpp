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
  int N(3), CAP(4);
  
  CPPL::zhsmatrix A(N,CAP);
  A.put(0,0, complex<double>(1.,0) );
  A.put(1,0, complex<double>(3.,4.) );
  A.put(1,1, complex<double>(2.,0.) );
  A.put(2,1, complex<double>(7.,8.) );
  cout << "A =\n" << A << endl;
  
  CPPL::zcovector x(N);
  x(0)=complex<double>(1,1);
  x(1)=complex<double>(2,2);
  x(2)=complex<double>(3,3);
  cout << "x =\n" << x << endl;
  
  cout << "A*x =\n" << A*x << endl;
  
  return 0;
}

/*****************************************************************************/
