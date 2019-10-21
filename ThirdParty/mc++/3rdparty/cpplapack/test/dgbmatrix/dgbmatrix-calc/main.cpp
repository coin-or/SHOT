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
  int M(3), N(4), KL(2), KU(1);
  
  CPPL::dgbmatrix A(M,N,KL,KU);
  for(int i=0; i<A.m; i++){
    for(int j=std::max(0l,i-A.kl); j<std::min(A.n,i+A.ku+1); j++){
      A(i,j) =double( rand() /(RAND_MAX/10) );
    }
  }
  
  cout << "A =\n" << A << endl;
  
  cout << "#### t(A) ####" << endl;
  cout << "t(A) =\n" << CPPL::t(A) << endl;
  
  cout << "#### i(A) ####" << endl;
  A.resize(M,M,KL,KU);
  for(int i=0; i<A.m; i++){ for(int j=0; j<A.n; j++){
    if( !((i-j)>A.kl || (j-i)>A.ku) ){ A(i,j) =double( rand() /(RAND_MAX/10) ); }
  }}
  CPPL::dgematrix A_inv;
  A_inv =CPPL::i(A);
  //A_inv =i(A); // g++ cannot compile this line
  
  cout << "A =\n" << A << endl;
  cout << "A_inv =\n" << A_inv << endl;
  cout << "A*A_inv =\n" << A*A_inv << endl;
  cout << "A_inv*A =\n" << A_inv*A << endl;
  
  return 0;
}

/*****************************************************************************/
