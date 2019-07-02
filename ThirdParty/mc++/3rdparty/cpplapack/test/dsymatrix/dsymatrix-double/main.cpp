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
  int N(3);
  
  CPPL::dsymatrix A(N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<=i; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  cout << "A =\n" << A << endl;
  cout << "A*10. =\n" << A*10. << endl;
  cout << "A/10. =\n" << A/10. << endl;
  
  cout << "#### A*=10.; ####" << endl;
  A*=10.;
  cout << "A =\n" << A << endl;
  cout << "#### A/=10.; ####" << endl;
  A/=10.;
  cout << "A =\n" << A << endl;
  
  return 0;
}

/*****************************************************************************/
