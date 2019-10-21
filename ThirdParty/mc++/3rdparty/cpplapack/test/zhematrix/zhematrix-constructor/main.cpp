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
  
  CPPL::zhematrix A;
  cout << "A || n=" << A.n << " array=" << A.array << " " << endl;
  
  
  CPPL::zhematrix B(N);
  for(int i=0; i<B.n; i++){
    for(int j=0; j<i; j++){
      B(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    B(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  cout << "B || n=" << B.n << " array=" << B.array << " " << endl;
  cout << B << endl;
  
  CPPL::zhematrix C(B);
  cout << "C || n=" << C.n << " array=" << C.array << " " << endl;
  cout << C << endl;
  
  return 0;
}

/*****************************************************************************/
