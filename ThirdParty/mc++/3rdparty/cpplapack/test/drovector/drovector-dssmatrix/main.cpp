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
  int N(5), CAP(4);
  
  CPPL::dssmatrix A(N,CAP);
  A.put(0,0, 1.);
  A.put(3,2, 2.);
  A.put(1,2, 3.);
  A.put(4,1, 4.);
  cout << "A =\n" << A << endl;
  A.checkup();
  
  CPPL::drovector x(N);
  for(int i=0; i<x.l; i++){
    x(i) =double( rand() /(RAND_MAX/10) );
  }
  cout << "x =\n" << x << endl;
  
  cout << "x*A =\n" << x*A << endl;
  
  CPPL::dsymatrix B( A.to_dsymatrix() );
  cout << "B =\n" << B << endl;
  cout << "x*B =\n" << x*B << endl;
  
  return 0;
}

/*****************************************************************************/
