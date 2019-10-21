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
  int M(3), N(4);
  
  CPPL::zhematrix A(N);
  for(int i=0; i<A.n; i++){
    for(int j=0; j<i; j++){
      A(i,j) =complex<double>(rand()/(RAND_MAX/10), rand()/(RAND_MAX/10));
    }
    A(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  
  cout << "A =\n" << A << endl;
  
  cout << "#### t(A) ####" << endl;
  cout << "t(A) =\n" << CPPL::t(A) << endl;

  cout << "#### i(A) ####" << endl;
  A.resize(M);
  for(int i=0; i<A.n; i++){
    for(int j=0; j<i; j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
    }
    A(i,i) =complex<double>(rand()/(RAND_MAX/10), 0.0);
  }
  CPPL::zgematrix A_inv;
  
  A_inv = CPPL::i(A);
  //A_inv =i(A); // g++ cannot compile this line
  
  cout << "A =\n" << A << endl;
  cout << "A_inv =\n" << A_inv << endl;
  cout << "A*A_inv =\n" << A*A_inv << endl;
  
  return 0;
}

/*****************************************************************************/
