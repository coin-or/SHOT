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
  
  CPPL::zhematrix A(N), B(N);
  for(int i=0; i<A.n; i++){
    for(int j=0; j<i; j++){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
      B(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    A(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
    B(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  cout << "A =\n" << A << endl;
  cout << "B =\n" << B << endl;
  
  cout << "A+B =\n" << A+B << endl;
  cout << "A-B =\n" << A-B << endl;
  cout << "A*B =\n" << A*B << endl;
  
  return 0;
}

/*****************************************************************************/
