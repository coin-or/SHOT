/*****************************************************************************/
/*                                 noname                                    */
/*****************************************************************************/

//=============================================================================
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "cpplapack.h"
using namespace std;

//=============================================================================
/*! main */
int main(int argc, char** argv)
{
  srand(time(NULL));
  int M(3), N(4);
  
  CPPL::dsymatrix A(N);
  for(int i=0; i<A.n; i++){ for(int j=0; j<=i; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  
  cout << "A =\n" << A << endl;
  
  cout << "#### t(A) ####" << endl;
  cout << "t(A) =\n" << CPPL::t(A) << endl;

  cout << "#### i(A) ####" << endl;
  A.resize(M);
  for(int i=0; i<A.n; i++){ for(int j=0; j<=i; j++){
    A(i,j) =double( rand() /(RAND_MAX/10) );
  }}
  CPPL::dsymatrix A_inv;
  
  A_inv = CPPL::i(A);
  //A_inv =i(A); // g++ cannot compile this line
  
  cout << "A =\n" << A << endl;
  cout << "A_inv =\n" << A_inv << endl;
  cout << "A*A_inv =\n" << A*A_inv << endl;
  
  return 0;
}

/*****************************************************************************/
