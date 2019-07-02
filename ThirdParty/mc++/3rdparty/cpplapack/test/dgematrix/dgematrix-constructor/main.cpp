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
  
  CPPL::dgematrix A;
  cout << "A || m=" << A.m << " n=" << A.n << " array=" << A.array << endl;
  
  
  CPPL::dgematrix B(M,N);
  for(int i=0; i<B.m; i++){ for(int j=0; j<B.n; j++){
    B(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  cout << "B || m=" << B.m << " n=" << B.n << " array=" << B.array << endl;
  cout << B << endl;
  
  CPPL::dgematrix C(B);
  cout << "C || m=" << C.m << " n=" << C.n << " array=" << C.array << endl;
  cout << C << endl;
  
  return 0;
}

/*****************************************************************************/
