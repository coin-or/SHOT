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
  
  CPPL::dsymatrix A;
  cout << "A || n=" << A.n << " array=" << A.array << " " << endl;
  
  
  CPPL::dsymatrix B(N);
  for(int i=0; i<B.n; i++){ for(int j=0; j<=i; j++){
    B(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  cout << "B || n=" << B.n << " array=" << B.array << " " << endl;
  cout << B << endl;
  
  CPPL::dsymatrix C(B);
  cout << "C || n=" << C.n << " array=" << C.array << " " << endl;
  cout << C << endl;
  
  return 0;
}

/*****************************************************************************/
