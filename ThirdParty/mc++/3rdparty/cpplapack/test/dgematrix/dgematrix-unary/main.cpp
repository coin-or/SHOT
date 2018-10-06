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
  int M(4), N(3);
  
  CPPL::dgematrix A(M,N);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
	A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  cout << "A =\n" << A << endl;
  cout << "+A =\n" << +A << endl;
  cout << "-A =\n" << -A << endl;
  cout << "A =\n" << A << endl;

  return 0;
}

/*****************************************************************************/
