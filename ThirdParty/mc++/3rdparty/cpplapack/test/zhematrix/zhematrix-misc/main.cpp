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
  
  CPPL::zhematrix A(N), B;
  for(int i=0; i<A.n; i++){
    for(int j=0; j<i; j++){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    A(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  cout << "A =\n" << A << endl;
  cout << "#### B.copy(A) ####" << endl;
  B.copy(A);
  cout << "B =\n" << B << endl;
  
  cout << "#### B.clear() ####" << endl;
  B.clear();
  cout << "B =\n" << B << endl;
  
  cout << "#### B.resize(2) & B.zero() ####" << endl;
  B.resize(2);
  B.zero();
  cout << "B =\n" << B << endl;
  
  cout << "#### B.identity() ####" << endl;
  B.identity();
  cout << "B =\n" << B << endl;
  
  return 0;
}

/*****************************************************************************/
